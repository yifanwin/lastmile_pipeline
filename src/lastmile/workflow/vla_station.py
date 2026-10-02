"""Case1 全 XY 起点视觉移动抓取；独立运行，不写六阶段 manifest。"""
import copy
import gzip
import json
import os
from pathlib import Path
import socket
import subprocess
import time
import traceback

import numpy as np
from . import core as c
from ..vla import CAMERAS, VLAValidator, action_vector, apply_vla_action, observation, sample_vla, select_xy_starts


def frozen_inputs(case_id, station_run_id, checkpoint, config):
    case = c.case_dir(case_id)
    station = case/c.STAGES[2]/'runs'/station_run_id
    source = case/c.STAGES[0]
    paths = [station/'candidates.json', station/'floor_grid.npz', station/'physics.json', station/'evidence_manifest.json',
             source/'compiled_model.mjb', source/'initial_snapshot.npz', source/'human_review.json',
             case/'inputs/source_episode.json', case/'inputs/engine_context/registry/episodes/0014.json',
             c.WORKSPACE/'configs/vla_station_test.yaml', c.WORKSPACE/'configs/protocols/protocol_v1.json',
             checkpoint/'config.yaml', checkpoint/'model.pt']
    paths += [c.WORKSPACE/'src/lastmile'/name for name in ('vla.py', 'vla_server.py', 'simulation.py', 'protocol.py',
               'workflow/vla_station.py', 'workflow/vla_station_report.py', 'workflow/core.py', 'workflow/station_geometry.py')]
    paths.append(c.WORKSPACE/'tests/physics/test_vla.py')
    paths.append(c.WORKSPACE/'tests/workflow/test_vla_server.py')
    upstream = c.WORKSPACE.parent/'MolmoBot/MolmoBot'
    paths += [upstream/name for name in ('olmo/eval/real_robot_molmobot_rby1_door.py',
              'olmo/eval/real_robot_molmobot_rby1_multitask.py', 'olmo/models/molmobot/inference_wrapper.py',
              'olmo/data/image_warping_utils.py', 'olmo/eval/websocket_server.py')]
    print('Freezing checkpoint and source hashes (20 GB model; NAS read may take time)', flush=True)
    return {'case_id': case_id, 'station_run_id': station_run_id, 'config': config,
            'files': {str(p.resolve()): c.digest(p) for p in paths},
            'git_branch': subprocess.check_output(['git', '-C', str(c.WORKSPACE), 'branch', '--show-current'], text=True).strip(),
            'git_commit': subprocess.check_output(['git', '-C', str(c.WORKSPACE), 'rev-parse', 'HEAD'], text=True).strip(),
            'semantics': 'full_20D_visual_mobile_pick_from_XY_not_fixed_base_reachability',
            'expectation': config['expectation']}


def require_frozen_match(saved, current):
    if saved != current:
        raise ValueError('冻结配置／代码／权重／来源改变，拒绝续跑；使用新的 run-id')


class PolicyService:
    def __init__(self, checkpoint, folder, config):
        import msgpack_numpy
        import websockets.sync.client
        sock = socket.socket()
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
        sock.close()
        upstream = c.WORKSPACE.parent/'MolmoBot/MolmoBot'
        env = dict(os.environ, HF_HUB_OFFLINE='1', TRANSFORMERS_OFFLINE='1',
                   PYTHONPATH=str(c.WORKSPACE/'src')+':'+str(upstream), OMP_NUM_THREADS='8')
        self.log = open(folder/'server.log', 'a')
        self.process = subprocess.Popen([str(upstream/'.venv/bin/python'), '-m', 'lastmile.vla_server',
            '--checkpoint-path', str(checkpoint), '--port', str(port), '--evidence-dir', str(folder/'trials'),
            '--seed', str(config['seed'])], env=env, cwd=upstream, stdout=self.log, stderr=subprocess.STDOUT)
        self.connection = None
        self.pack, self.unpack = msgpack_numpy.packb, msgpack_numpy.unpackb
        deadline = time.monotonic()+config['server_start_timeout_s']
        try:
            while True:
                if self.process.poll() is not None:
                    raise RuntimeError('模型服务启动失败，见 server.log')
                try:
                    self.connection = websockets.sync.client.connect(f'ws://127.0.0.1:{port}', compression=None,
                        max_size=None, open_timeout=5, ping_interval=None, proxy=None)
                    break
                except OSError:
                    if time.monotonic() > deadline:
                        raise TimeoutError('模型服务启动超时')
                    time.sleep(2)
            self.metadata = self.unpack(self.connection.recv(timeout=10))
            required = {'checkpoint_path': str(checkpoint.resolve()), 'config_sha256': c.digest(checkpoint/'config.yaml'),
                        'action_dim': 20, 'state_dim': 22, 'camera_names': list(CAMERAS), 'clamp_gripper': False,
                        'action_horizon': 16, 'execute_horizon': 8, 'seed': config['seed']}
            if any(self.metadata.get(k) != v for k, v in required.items()):
                raise RuntimeError('模型服务 metadata 不匹配')
        except BaseException:
            self.close()
            raise
        self.timeout = config['inference_timeout_s']

    def infer(self, obs):
        self.connection.send(self.pack(obs))
        reply = self.connection.recv(timeout=self.timeout)
        if isinstance(reply, str):
            raise RuntimeError('推理服务异常：'+reply[-2000:])
        action = self.unpack(reply)
        action_vector(action)
        return action

    def close(self):
        if self.connection:
            self.connection.close()
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        self.log.close()


def place_base_only(sim, base):
    import mujoco
    sim.restore()
    before_qpos, before_qvel = sim.data.qpos.copy(), sim.data.qvel.copy()
    allowed_qpos, allowed_dof = [], []
    for name, value in zip(('base_x', 'base_y', 'base_theta'), base):
        adr = sim.qpos_adr[name]
        dof = int(sim.model.jnt_dofadr[sim.model.joint('robot_0/'+name).id])
        sim.data.qpos[adr], sim.data.qvel[dof] = value, 0.
        allowed_qpos.append(adr)
        allowed_dof.append(dof)
    sim.hold_ctrl()
    mujoco.mj_forward(sim.model, sim.data)
    qmask = np.ones(sim.model.nq, bool)
    vmask = np.ones(sim.model.nv, bool)
    qmask[allowed_qpos], vmask[allowed_dof] = False, False
    unchanged = np.array_equal(sim.data.qpos[qmask], before_qpos[qmask]) and np.array_equal(sim.data.qvel[vmask], before_qvel[vmask])
    if not unchanged:
        raise RuntimeError('初始化修改了非底盘状态')
    sim.vla_head_reference = sim.group('head').copy()
    return {'only_robot_base_changed': True, 'all_nonbase_qpos_and_qvel_unchanged': bool(unchanged),
            'base': list(base), 'initial_time_s': float(sim.data.time)}


def configure_cameras(sim, config):
    import mujoco
    cameras = []
    for name in CAMERAS:
        index = sim.model.camera('robot_0/'+name).id
        sim.model.cam_fovy[index] = config['camera_fov_deg'][name]
        cameras.append({'name': name, 'fovy': float(sim.model.cam_fovy[index]), 'position': sim.model.cam_pos[index].tolist(),
                        'quaternion': sim.model.cam_quat[index].tolist(), 'body_id': int(sim.model.cam_bodyid[index])})
    sim.model.vis.global_.offwidth = max(1024, sim.model.vis.global_.offwidth)
    sim.model.vis.global_.offheight = max(576, sim.model.vis.global_.offheight)
    mujoco.mj_forward(sim.model, sim.data)
    return cameras


def render_images(renderer, sim):
    images = {}
    for name in CAMERAS:
        renderer.update_scene(sim.data, camera='robot_0/'+name)
        images[name] = renderer.render().copy()
    return images


def trial(sim, renderer, observer_camera, service, row, folder, config, thresholds):
    import imageio.v2 as imageio
    import mujoco
    from PIL import Image
    initial = sample_vla(sim)
    c.write(folder/'initial.json', initial)
    state = np.zeros_like(sim.initial_state)
    mujoco.mj_getState(sim.model, sim.data, state, sim.state_spec)
    np.savez_compressed(folder/'placed_snapshot.npz', state=state)
    validator = VLAValidator(initial, thresholds, config['torso_feedback_policy'],
                             config['collision_feedback_policy'], config['severe_penetration_m'])
    times, states = [float(sim.data.time)], [sim.data.qpos.copy()]
    actions, verdict = [], None
    def forbid_restore():
        raise RuntimeError('VLA 连续执行期间禁止 restore')
    old_restore = sim.restore
    sim.restore = forbid_restore
    frames = 0
    writer = imageio.get_writer(str(folder/'rollout.mp4'), fps=config['video_fps'], codec='libx264', macro_block_size=16)
    start = time.monotonic()
    try:
        with gzip.open(folder/'trace.jsonl.gz', 'wt', encoding='utf-8') as trace:
            for step in range(config['task_horizon']):
                images = render_images(renderer, sim)
                if step == 0:
                    for name, img in images.items():
                        Image.fromarray(img).save(folder/(name+'_initial.png'))
                renderer.update_scene(sim.data, camera=observer_camera)
                third = renderer.render().copy()
                panels = [images['head_camera'], third, images['wrist_camera_l'], images['wrist_camera_r']]
                panels = [np.asarray(Image.fromarray(im).resize((512, 288))) for im in panels]
                writer.append_data(np.vstack((np.hstack(panels[:2]), np.hstack(panels[2:]))))
                frames += 1
                obs = observation(sim, images, config['task'], reset=step == 0)
                obs['_evidence_key'] = folder.name
                before = time.monotonic()
                action = service.infer(obs)
                record = apply_vla_action(sim, action)
                record.update(step=step, time_s=float(sim.data.time), inference_wall_s=time.monotonic()-before,
                              server_timing=action.get('server_timing'))
                actions.append(record)
                # 5 个控制周期；每周期 5 个物理子步。位置伺服目标保持，不重复加 delta。
                for _ in range(5):
                    for _ in range(5):
                        mujoco.mj_step(sim.model, sim.data)
                        # mj_step 后位置相关缓存需显式同步，避免采样读取上一 tick。
                        mujoco.mj_forward(sim.model, sim.data)
                        sample = sample_vla(sim)
                        trace.write(json.dumps(sample, allow_nan=False, separators=(',', ':'))+'\n')
                        previous_warning_types = set(validator.torso_warnings)
                        previous_collision_types = set(validator.collision_warnings)
                        verdict = validator.feed(sample)
                        for warning in set(validator.collision_warnings)-previous_collision_types:
                            print('WARNING', row['pose_id'], warning, validator.collision_warnings[warning],
                                  '轻微碰撞仅告警，继续 episode', flush=True)
                        for warning in set(validator.torso_warnings)-previous_warning_types:
                            print('WARNING', row['pose_id'], warning, validator.torso_warnings[warning],
                                  '反馈偏移仅告警，继续 episode', flush=True)
                        if verdict:
                            break
                    if verdict:
                        break
                times.append(float(sim.data.time))
                states.append(sim.data.qpos.copy())
                if verdict:
                    break
            verdict = verdict or validator.finish()
    finally:
        sim.restore = old_restore
        writer.close()
        c.write(folder/'actions.json', actions)
        np.savez_compressed(folder/'states.npz', times=np.asarray(times), qpos=np.asarray(states))
    result = {**verdict, 'pose_id': row['pose_id'], 'initial_base': row['base'], 'final_base': sim.group('base').tolist(),
              'policy_steps': len(actions), 'sim_duration_s': float(sim.data.time)-initial['time_s'], 'video_frames': frames,
              'wall_duration_s': time.monotonic()-start, 'limited_action_steps': sum(bool(a['limits']) for a in actions),
              'binary_pick_label': int(verdict['status'] == 'verified_success') if verdict['status'] != 'infrastructure_error' else None}
    c.write(folder/'result.json', result)
    return result


def audit_trial(folder, thresholds, torso_feedback_policy='strict',
                collision_feedback_policy='strict', severe_penetration_m=.02):
    initial = c.read(folder/'initial.json')
    validator = VLAValidator(initial, thresholds, torso_feedback_policy, collision_feedback_policy, severe_penetration_m)
    count = 0
    with gzip.open(folder/'trace.jsonl.gz', 'rt', encoding='utf-8') as f:
        for line in f:
            if validator.terminal:
                raise ValueError('terminal verdict 之后仍有轨迹')
            validator.feed(json.loads(line))
            count += 1
    recomputed = validator.finish()
    recorded = c.read(folder/'result.json')
    if any(recomputed.get(k) != recorded.get(k) for k in ('status', 'reason', 'selected_arm')):
        raise ValueError('保存轨迹重新验收与结果不一致')
    if collision_feedback_policy == 'warn_minor' and recomputed.get('collision_warnings') != recorded.get('collision_warnings'):
        raise ValueError('碰撞警告重放与结果不一致')
    if torso_feedback_policy == 'warn':
        if recomputed.get('torso_feedback_warnings') != recorded.get('torso_feedback_warnings'):
            raise ValueError('反馈警告重放与结果不一致')
        actions = c.read(folder/'actions.json')
        if not actions:
            raise ValueError('缺失派发动作证据')
        for action in actions:
            h = action['torso_command_h']
            expected = [0., h, -2*h, h, 0., 0.]
            actual = [action['applied_targets'][f'link{i+1}_act'] for i in range(6)]
            if (not 0. <= h <= .738 or action.get('torso_command_verified') is not True
                or not np.isfinite(actual).all() or not np.allclose(actual, expected, atol=1e-12, rtol=0)
                or action.get('torso_command_error_rad') != float(np.max(np.abs(np.array(actual)-expected)))):
                raise ValueError('torso_command_linkage: 派发证据不符合映射')
    return {'samples': count, 'recomputed': recomputed, 'trace_sha256': c.digest(folder/'trace.jsonl.gz')}


def run_vla_station_test(case_id='case1-cup-001', station_run_id='coarse_v1', run_id='molmobot_multitask_xy_v1', checkpoint_path=None, resume=False):
    c.case_dir(run_id)
    c.case_dir(station_run_id)
    c.require_next_stage(case_id)
    case = c.case_dir(case_id)
    episode = c.read(case/'inputs/source_episode.json')
    if episode['provenance']['source_index'] != 14:
        raise ValueError('当前接入仅支持已审计的 case1 episode 14')
    config = c.load_config('vla_station_test.yaml')
    checkpoint = Path(checkpoint_path or config['checkpoint_path']).resolve()
    config['checkpoint_path'] = str(checkpoint)
    folder = case/c.STAGES[2]/'runs'/run_id
    if folder.exists() and not resume:
        raise ValueError('不覆盖旧 run，使用 --resume 或新的 run-id')
    if resume and not (folder/'frozen_inputs.json').exists():
        raise ValueError('没有可续跑的冻结 run')
    frozen = frozen_inputs(case_id, station_run_id, checkpoint, config)
    if resume:
        require_frozen_match(c.read(folder/'frozen_inputs.json'), frozen)
    else:
        folder.mkdir(parents=True, exist_ok=False)
        c.write(folder/'frozen_inputs.json', frozen)
    station = case/c.STAGES[2]/'runs'/station_run_id
    selected = select_xy_starts(c.read(station/'candidates.json'))
    c.write(folder/'selected_starts.json', selected)
    thresholds = c.read(c.WORKSPACE/'configs/protocols/protocol_v1.json')['thresholds']
    protocol = {'version': 'lastmile-visual-mobile-pick-v3', 'thresholds': thresholds,
                'base_fixed': False, 'idle_arm_fixed': False, 'head_fixed': True,
                'torso_actual_linkage_checked': True, 'torso_feedback_policy': config['torso_feedback_policy'],
                'torso_command_linkage_required': True, 'torso_feedback_deviations_are_terminal': config['torso_feedback_policy'] == 'strict',
                'physical_joint_limits_remain_terminal': True,
                'collision_feedback_policy': config['collision_feedback_policy'],
                'severe_penetration_m': config['severe_penetration_m'],
                'initial_collision_filter_unchanged': True, 'joint_limit_tolerance_rad': .002,
                'success': 'either_same_gripper_two_force_bearing_fingers_lift_5cm_unsupported_hold_2s',
                'legacy_pick_protocol_unchanged': True}
    c.write(folder/'vla_protocol.json', protocol)
    results = c.read(folder/'results.json') if (folder/'results.json').exists() else []
    completed = {r['pose_id']: r for r in results if r['status'] in ('verified_success', 'executed_failure', 'invalid_initialization')}
    for record in completed.values():
        if record.get('attempt'):
            dest = folder/'trials'/record['attempt']
            manifest = c.read(dest/'evidence_manifest.json')
            for path, sha in manifest['files'].items():
                if c.digest(dest/path) != sha:
                    raise ValueError('续跑证据被篡改：'+path)
            audit_trial(dest, thresholds, config['torso_feedback_policy'],
                                    config['collision_feedback_policy'], config['severe_penetration_m'])
    results = [r for r in results if r['pose_id'] in completed]
    c.write(folder/'results.json', results)
    from ..simulation import Simulation
    from .station_geometry import initial_collision
    import mujoco
    sim = None
    renderer = service = None
    try:
        sim = Simulation.from_cached(case/'inputs/engine_context', 14, case/c.STAGES[0], folder/'engine')
        if not np.isclose(sim.model.opt.timestep, config['sim_dt_s']):
            raise ValueError('物理 timestep 与冻结配置不符')
        cameras = configure_cameras(sim, config)
        c.write(folder/'camera_calibration.json', {'cameras': cameras, 'randomization_enabled': False, 'camera_only_changes': True})
        grid = np.load(station/'floor_grid.npz')
        valid = []
        for row in selected:
            if row['pose_id'] in completed:
                continue
            placement = place_base_only(sim, row['base'])
            collision = initial_collision(sim)
            ix = int(np.argmin(abs(grid['xs']-row['base'][0])))
            iy = int(np.argmin(abs(grid['ys']-row['base'][1])))
            on_grid = grid['xs'][0] <= row['base'][0] <= grid['xs'][-1] and grid['ys'][0] <= row['base'][1] <= grid['ys'][-1]
            covered = bool(on_grid and grid['clearance'][ix, iy] >= .34)
            if collision or not covered:
                results.append({'pose_id': row['pose_id'], 'initial_base': row['base'], 'status': 'invalid_initialization',
                    'reason': 'initial_pose_collision' if collision else 'floor_footprint_rejected', 'initial_collision': collision,
                    'floor_footprint_covered': covered, 'placement': placement, 'binary_pick_label': None})
            else:
                valid.append(row)
        c.write(folder/'results.json', results)
        print('XY records', len(selected), 'pending valid trials', len(valid), 'invalid', sum(r['status']=='invalid_initialization' for r in results), flush=True)
        renderer = mujoco.Renderer(sim.model, height=576, width=1024)
        facts = c.read(case/c.STAGES[0]/'scene_facts.json')
        camera = mujoco.MjvCamera()
        camera.lookat[:] = facts['support_AABB_center']
        camera.distance, camera.azimuth, camera.elevation = 4.5, 150, -45
        # 相机预检先于模型启动，失败不冒充模型抓取失败。
        if valid:
            place_base_only(sim, valid[0]['base'])
            observation(sim, render_images(renderer, sim), config['task'])
            service = PolicyService(checkpoint, folder, config)
            c.write(folder/'preflight.json', {'cuda_and_model_ready': True, 'camera_ready': True, 'offline': True,
                    'service_metadata': service.metadata, 'MUJOCO_GL': os.environ.get('MUJOCO_GL'),
                    'CUDA_VISIBLE_DEVICES': os.environ.get('CUDA_VISIBLE_DEVICES'), 'completed_at_utc': c.now()})
        for number, row in enumerate(valid):
            existing = sorted((folder/'trials').glob(row['pose_id']+'_attempt_*'))
            attempt = f'{row["pose_id"]}_attempt_{len(existing):03d}'
            dest = folder/'trials'/attempt
            dest.mkdir(parents=True, exist_ok=False)
            placement = place_base_only(sim, row['base'])
            c.write(dest/'placement.json', placement)
            print('VLA', number+1, '/', len(valid), row['pose_id'], 'starting', flush=True)
            try:
                result = trial(sim, renderer, camera, service, row, dest, config, thresholds)
                audit = audit_trial(dest, thresholds, config['torso_feedback_policy'],
                                    config['collision_feedback_policy'], config['severe_penetration_m'])
                c.write(dest/'audit.json', audit)
                # 非底盘状态与源快照再次从保存快照核对。
                data = mujoco.MjData(sim.model)
                mujoco.mj_setState(sim.model, data, np.load(dest/'placed_snapshot.npz')['state'], sim.state_spec)
                mask = np.ones(sim.model.nq, bool)
                mask[[sim.qpos_adr[n] for n in ('base_x', 'base_y', 'base_theta')]] = False
                if not np.array_equal(data.qpos[mask], sim.initial_qpos[mask]):
                    raise ValueError('保存快照非底盘状态与来源不一致')
                reference = mujoco.MjData(sim.model)
                mujoco.mj_setState(sim.model, reference, sim.initial_state, sim.state_spec)
                vmask = np.ones(sim.model.nv, bool)
                vmask[[int(sim.model.jnt_dofadr[sim.model.joint('robot_0/'+n).id]) for n in ('base_x', 'base_y', 'base_theta')]] = False
                if not np.array_equal(data.qvel[vmask], reference.qvel[vmask]):
                    raise ValueError('保存快照非底盘速度与来源不一致')
                files = [p for p in dest.rglob('*') if p.is_file()]
                c.write(dest/'evidence_manifest.json', {'files': {str(p.relative_to(dest)): c.digest(p) for p in files}})
                record = {**result, 'attempt': attempt, 'audit_passed': True}
            except BaseException as exc:
                c.write(dest/'interruption.json', {'error_type': type(exc).__name__, 'error': str(exc), 'traceback': traceback.format_exc(), 'at_utc': c.now()})
                record = {'pose_id': row['pose_id'], 'initial_base': row['base'], 'status': 'infrastructure_error',
                          'reason': type(exc).__name__, 'attempt': attempt, 'binary_pick_label': None}
                results.append(record)
                c.write(folder/'results.json', results)
                raise
            results.append(record)
            c.write(folder/'results.json', results)
            print('VLA', row['pose_id'], record['status'], record['reason'], record['sim_duration_s'], flush=True)
            if record['status'] == 'infrastructure_error':
                raise RuntimeError('逐步验收出现基础设施异常，停止公共批次')
        c.write(folder/'run_state.json', {'status': 'complete', 'completed_at_utc': c.now()})
    except BaseException as exc:
        c.write(folder/'run_state.json', {'status': 'infrastructure_error', 'error_type': type(exc).__name__, 'error': str(exc),
                                        'traceback': traceback.format_exc(), 'at_utc': c.now()})
        raise
    finally:
        if service:
            service.close()
        if renderer:
            renderer.close()
        from .vla_station_report import export_vla_report
        export_vla_report(case_id, station_run_id, run_id)
    return c.read(folder/'summary.json')

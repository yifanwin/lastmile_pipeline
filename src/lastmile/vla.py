"""独立全 20 维 VLA 控制、观测和验收；不改变旧 Pick 协议。"""
import copy
import numpy as np
from .protocol import ORDER, finite_vector, torso_joints
from .validation import rotation_distance

CAMERAS = ('wrist_camera_r', 'head_camera', 'wrist_camera_l')


def select_xy_starts(rows):
    selected = [copy.deepcopy(r) for r in rows if r.get('is_A') or r.get('yaw_offset_deg') == 0]
    selected.sort(key=lambda r: (not r.get('is_A', False), r['pose_id']))
    xy = [tuple(finite_vector(r['base'], 3)[:2]) for r in selected]
    if len(set(xy)) != len(xy) or len({r['pose_id'] for r in selected}) != len(selected):
        raise ValueError('候选包含重复 XY 或 pose_id')
    if not selected or selected[0]['pose_id'] != 'A':
        raise ValueError('必须包含精确 A 点')
    return selected


def observation(sim, images, task, reset=False):
    if tuple(images) != CAMERAS:
        raise ValueError('相机顺序不匹配')
    for image in images.values():
        if image.shape != (576, 1024, 3) or image.dtype != np.uint8:
            raise ValueError('需要 1024×576 RGB uint8')
    qpos = {name: sim.group(name).astype(np.float32) for name in ('base', 'left_arm', 'right_arm', 'torso')}
    for side in ('left', 'right'):
        qpos[side+'_gripper'] = sim.q([f'gripper_finger_{side[0]}1']).astype(np.float32)
    state = np.concatenate([qpos[name][[1, 2, 3]] if name == 'torso' else qpos[name] for name, _ in ORDER])
    finite_vector(state, 22)
    return {**images, 'qpos': qpos, 'task': task, 'reset': reset}


def action_vector(action):
    return np.concatenate([finite_vector(action[name], size) for name, size in ORDER])


def apply_vla_action(sim, action):
    """相对目标只在策略 tick 求一次，物理子步不重新累加。"""
    raw = action_vector(action)
    proposed, controls = {}, {}
    for side in ('left', 'right'):
        for i, value in enumerate(sim.group(side+'_arm') + action[side+'_arm']):
            proposed[f'{side}_arm_{i+1}_act'] = float(value)
        proposed[side+'_finger_act'] = float(action[side+'_gripper'][0])
    for name, value in zip(('base_x', 'base_y', 'base_theta'), sim.group('base') + action['base']):
        proposed[name+'_act'] = float(value)
    raw_h = float(action['torso'][0])
    h = float(np.clip(raw_h, 0., .738))
    for i, value in enumerate(torso_joints(h)):
        proposed[f'link{i+1}_act'] = float(value)
    for i, value in enumerate(sim.vla_head_reference):
        proposed[f'head_{i}_act'] = float(value)
    limited = {}
    if h != raw_h:
        limited['torso_height'] = {'raw': raw_h, 'applied': h}
    for name, value in proposed.items():
        index = sim.act[name]
        target = float(np.clip(value, *sim.model.actuator_ctrlrange[index])) if sim.model.actuator_ctrllimited[index] else value
        if target != value:
            limited[name] = {'raw': value, 'applied': target}
        controls[name] = target
    dispatched = np.array([controls[f'link{i+1}_act'] for i in range(6)])
    command_error = float(np.max(np.abs(dispatched-torso_joints(h))))
    if not np.isfinite(dispatched).all() or command_error > 1e-12:
        raise ValueError('torso_command_linkage: 执行器限幅破坏躯干派发映射；未派发任何动作')
    for name, target in controls.items():
        sim.data.ctrl[sim.act[name]] = target
    return {'torso_command_verified': True, 'torso_command_h': h, 'torso_command_error_rad': command_error,
            'raw_action': raw.tolist(), 'proposed_targets': proposed, 'applied_targets': controls, 'limits': limited}


def sample_vla(sim):
    """同时采集两夹爪；允许双方指尖接触，但不允许掌部或其他身体接触目标。"""
    import mujoco
    from .simulation import body_pose
    m, d = sim.model, sim.data
    fingers = {s: {m.body(f'robot_0/ee_finger_{s[0]}{i}').id for i in (1, 2)} for s in ('left', 'right')}
    all_fingers = fingers['left'] | fingers['right']
    forces = {s: {} for s in fingers}
    illegal, illegal_contacts, robot_contacts, support = [], [], [], False
    def add_illegal(reason, contact, force):
        illegal.append(reason)
        illegal_contacts.append({'reason': reason, 'penetration_m': max(0., -float(contact.dist)),
                                 'geom1': int(contact.geom1), 'geom2': int(contact.geom2),
                                 'normal_force_N': float(force[0])})
    for ci, contact in enumerate(d.contact):
        force = np.zeros(6)
        mujoco.mj_contactForce(m, d, ci, force)
        bearing = bool(contact.efc_address >= 0 and force[0] > 1e-6)
        if contact.dist > 0 and not bearing:
            continue
        a, b = int(m.geom_bodyid[contact.geom1]), int(m.geom_bodyid[contact.geom2])
        ta, tb = int(m.body_rootid[a]) == sim.target_root, int(m.body_rootid[b]) == sim.target_root
        ra, rb = a in sim.robot_bodies, b in sim.robot_bodies
        if (ra or rb) and contact.dist < -.001:
            robot_contacts.append({'reason': m.geom(int(contact.geom1)).name+':'+m.geom(int(contact.geom2)).name,
                                   'penetration_m': -float(contact.dist),
                                   'geom1': int(contact.geom1), 'geom2': int(contact.geom2),
                                   'normal_force_N': float(force[0])})
        if ta != tb:
            other = b if ta else a
            if other in all_fingers and bearing:
                side = 'left' if other in fingers['left'] else 'right'
                forces[side][other] = forces[side].get(other, 0.) + float(force[0])
            elif other in sim.robot_bodies:
                if contact.dist < -.001:
                    add_illegal('nonfinger_target:'+m.body(other).name, contact, force)
            elif bearing:
                support = True
        elif ra and rb and a != b and contact.dist < -.002:
            add_illegal('self_collision:'+m.body(a).name+':'+m.body(b).name, contact, force)
        elif ra != rb and not (ta or tb) and contact.dist < -.001:
            other_geom = int(contact.geom2 if ra else contact.geom1)
            robot_body = a if ra else b
            floor = 'floor' in m.geom(other_geom).name.lower()
            base = any(x in m.body(robot_body).name for x in ('base', 'wheel'))
            if not (floor and base):
                add_illegal('environment:'+m.geom(other_geom).name, contact, force)
    arms = {}
    pose = body_pose(d, sim.target)
    for side in fingers:
        arms[side] = {'expected_fingers': sorted(fingers[side]), 'finger_normal_forces_N': forces[side],
                      'relative_pose': np.linalg.solve(sim.tcp(side), pose).tolist()}
    joints = {s: sim.group(s+'_arm').tolist() for s in fingers}
    joint_limits_ok = True
    for name in sim.qpos_adr:
        jid = m.joint('robot_0/'+name).id
        if m.jnt_limited[jid]:
            q = d.qpos[sim.qpos_adr[name]]
            lo, hi = m.jnt_range[jid]
            if not lo-.002 <= q <= hi+.002:
                joint_limits_ok = False
    return {'time_s': float(d.time), 'target_id': sim.target_name, 'base': sim.group('base').tolist(),
            'torso': sim.group('torso').tolist(), 'head': sim.group('head').tolist(), 'joints': joints,
            'height_m': float(d.xpos[sim.target][2]), 'target_pose': pose.tolist(),
            'support_contact': bool(support), 'illegal': sorted(set(illegal)), 'illegal_contacts': illegal_contacts, 'robot_contacts': robot_contacts, 'arms': arms,
            'joint_limits_ok': bool(joint_limits_ok), 'contact_sampler_version': 'vla-force-aware-v1'}


class VLAValidator:
    """增量验收；完整轨迹重放使用同一个状态机，不依赖模型声明成功。"""
    def __init__(self, initial, thresholds, torso_feedback_policy='strict',
                 collision_feedback_policy='strict', severe_penetration_m=.02):
        if torso_feedback_policy not in ('strict', 'warn'):
            raise ValueError('unknown torso_feedback_policy')
        if collision_feedback_policy not in ('strict', 'warn_minor'):
            raise ValueError('unknown collision_feedback_policy')
        if not np.isfinite(severe_penetration_m) or severe_penetration_m <= .002:
            raise ValueError('invalid severe_penetration_m')
        self.collision_feedback_policy = collision_feedback_policy
        self.severe_penetration_m = severe_penetration_m
        self.collision_warnings = {}
        self.torso_feedback_policy = torso_feedback_policy
        self.torso_warnings = {}
        self.initial, self.thresholds = initial, thresholds
        self.previous = float(initial['time_s'])
        self.windows = {s: None for s in ('left', 'right')}
        self.terminal = None

    def feed(self, row):
        if self.terminal:
            return self.terminal
        t = self.thresholds
        required = {'time_s', 'target_id', 'base', 'torso', 'head', 'height_m', 'support_contact', 'illegal', 'arms', 'joint_limits_ok', 'contact_sampler_version'}
        def end(status, reason, **extra):
            self.terminal = {'status': status, 'reason': reason, **extra}
            if self.torso_feedback_policy == 'warn':
                self.terminal['torso_feedback_warnings'] = copy.deepcopy(self.torso_warnings)
            if self.collision_feedback_policy == 'warn_minor':
                self.terminal['collision_warnings'] = copy.deepcopy(self.collision_warnings)
            return self.terminal
        if not required <= row.keys():
            return end('infrastructure_error', 'missing_trace_fields')
        try:
            time = float(row['time_s'])
            if not np.isfinite(time) or not self.previous < time <= self.previous+t['max_sample_gap_s']:
                return end('infrastructure_error', 'trace_gap_or_invalid_time')
            self.previous = time
            if row['contact_sampler_version'] != 'vla-force-aware-v1':
                return end('infrastructure_error', 'unsupported_contact_sampler')
            finite_vector(row['base'], 3)
            torso = finite_vector(row['torso'], 6)
            head = finite_vector(row['head'], 2)
            if not np.isfinite(row['height_m']):
                return end('infrastructure_error', 'nonfinite_state')
            if row['target_id'] != self.initial['target_id']:
                return end('executed_failure', 'wrong_target')
            if self.collision_feedback_policy == 'warn_minor':
                details = row['illegal_contacts']
                robot_details = row['robot_contacts']
                if not isinstance(robot_details, list):
                    return end('infrastructure_error', 'missing_collision_depth_evidence')
                if not isinstance(details, list) or {x['reason'] for x in details} != set(row['illegal']):
                    return end('infrastructure_error', 'missing_collision_depth_evidence')
                for contact in details+robot_details:
                    depth = float(contact['penetration_m'])
                    force = float(contact['normal_force_N'])
                    if not np.isfinite(depth) or depth < 0 or not np.isfinite(force):
                        return end('infrastructure_error', 'invalid_collision_depth_evidence')
                if any(x['penetration_m'] >= self.severe_penetration_m for x in details+robot_details):
                    return end('executed_failure', 'severe_penetration',
                               severe_contacts=[x for x in robot_details+details if x['penetration_m'] >= self.severe_penetration_m])
                for reason in set(row['illegal']):
                    depth = max(x['penetration_m'] for x in details if x['reason'] == reason)
                    warning = self.collision_warnings.setdefault(reason, {'samples': 0, 'first_time_s': time,
                                                                           'last_time_s': time, 'max_penetration_m': 0.})
                    warning['samples'] += 1
                    warning['last_time_s'] = time
                    warning['max_penetration_m'] = max(warning['max_penetration_m'], depth)
            elif row['illegal']:
                return end('executed_failure', 'illegal_collision')
            if row['joint_limits_ok'] is not True:
                return end('executed_failure', 'joint_limits')
            # 派发映射在动作接口严格检查；反馈偏移可按独立协议只告警。
            actual_h = float(torso[1])
            expected = np.array([0., actual_h, -2*actual_h, actual_h, 0., 0.])
            deviations = {'torso_range': max(0., -actual_h, actual_h-.738),
                          'torso_linkage': float(np.max(np.abs(torso-expected)))}
            for reason, residual in deviations.items():
                if residual <= t['torso_error_rad']:
                    continue
                if self.torso_feedback_policy == 'strict':
                    return end('executed_failure', reason)
                warning = self.torso_warnings.setdefault(reason, {'samples': 0, 'first_time_s': time,
                                                                   'last_time_s': time, 'max_residual_rad': 0.})
                warning['samples'] += 1
                warning['last_time_s'] = time
                warning['max_residual_rad'] = max(warning['max_residual_rad'], residual)
            if np.max(np.abs(head-finite_vector(self.initial['head'], 2))) > t['head_error_rad']:
                return end('executed_failure', 'head_protocol')
            # 先验证双臂所有字段再判成功，不能用另一臂的坏数据换成功。
            contacts = {}
            for side in self.windows:
                arm = row['arms'][side]
                expected_fingers = set(self.initial['arms'][side]['expected_fingers'])
                if set(arm['expected_fingers']) != expected_fingers or len(expected_fingers) != 2:
                    return end('infrastructure_error', 'finger_identity')
                forces = {int(k): float(v) for k, v in arm['finger_normal_forces_N'].items()}
                if not set(forces) <= expected_fingers or any(not np.isfinite(v) or v <= 1e-6 for v in forces.values()):
                    return end('infrastructure_error', 'invalid_contact_forces')
                pose = np.asarray(arm['relative_pose'], float)
                if (pose.shape != (4, 4) or not np.isfinite(pose).all() or not np.allclose(pose[3], [0, 0, 0, 1])
                    or not np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-5)
                    or not np.isclose(np.linalg.det(pose[:3, :3]), 1)):
                    return end('infrastructure_error', 'invalid_relative_pose')
                contacts[side] = (set(forces) == expected_fingers, pose)
            for side, (two_fingers, pose) in contacts.items():
                held = two_fingers and row['support_contact'] is False and row['height_m']-self.initial['height_m'] >= t['lift_m']
                if not held:
                    self.windows[side] = None
                    continue
                window = self.windows[side]
                if window is None:
                    self.windows[side] = (time, pose.copy())
                    continue
                start, reference = window
                if (np.linalg.norm(pose[:3, 3]-reference[:3, 3]) > t['relative_translation_m']
                    or rotation_distance(reference[:3, :3], pose[:3, :3]) > t['relative_rotation_rad']):
                    self.windows[side] = None
                    continue
                if time-start >= t['hold_s']-1e-9:
                    return end('verified_success', 'strict_visual_mobile_pick', selected_arm=side, hold_start_s=start, success_time_s=time)
        except (KeyError, TypeError, ValueError, IndexError, OverflowError):
            return end('infrastructure_error', 'invalid_trace_fields')
        return None

    def finish(self):
        if self.previous == self.initial['time_s']:
            return {'status': 'infrastructure_error', 'empty_trace' : True, 'reason': 'missing_trace_fields'}
        result = self.terminal or {'status': 'executed_failure', 'reason': 'horizon_exhausted_lift_contact_hold_or_slip'}
        if self.torso_feedback_policy == 'warn':
            result = {**result, 'torso_feedback_warnings': copy.deepcopy(self.torso_warnings)}
        if self.collision_feedback_policy == 'warn_minor':
            result = {**result, 'collision_warnings': copy.deepcopy(self.collision_warnings)}
        return result

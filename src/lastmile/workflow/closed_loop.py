"""步骤 5：独立派生起点 → A* 真动态导航 → 实测到站 cuRobo → 严格 Pick。"""
import copy,traceback
import numpy as np
from . import core as c
from .hard_start import require_robot_candidate,snapshot_base_only

def validate_navigation_rows(rows,initial,thresholds):
    from ..protocol import torso_joints
    if not rows:return 'empty_navigation_trace'
    previous=initial['time_s']
    for row in rows:
        t=row['time_s']
        if t<=previous or t-previous>.004001:return 'navigation_time_gap_or_reset'
        previous=t
        if row['illegal'] or row['forbidden_navigation_contact']:return 'navigation_collision'
        if np.max(np.abs(np.array(row['torso'])-torso_joints(0)))>thresholds['torso_error_rad']:return 'navigation_torso'
        if np.max(np.abs(np.array(row['head'])-initial['head']))>.002:return 'navigation_head'
        if np.max(np.abs(np.array(row['selected_arm_q'])-initial['left_arm']))>.002 or np.max(np.abs(np.array(row['idle_arm'])-initial['right_arm']))>.002:return 'navigation_idle_arm'
        if np.linalg.norm(np.array(row['target_pose'])[:3,3]-np.array(initial['target_pose'])[:3,3])>.002:return 'navigation_target_translation'
        from scipy.spatial.transform import Rotation
        if Rotation.from_matrix(np.array(initial['target_pose'])[:3,:3].T@np.array(row['target_pose'])[:3,:3]).magnitude()>.01:return 'navigation_target_rotation'
    return None

def run_closed_loop(case_id,construction_run_id,run_id,trials=3,seed_offset=0):
    c.case_dir(run_id);construction,config=require_robot_candidate(case_id,construction_run_id)
    if not 1<=trials<=config['config']['physics']['trials']:raise ValueError('trial count outside frozen budget')
    P=c.read(c.WORKSPACE/'configs/protocols/protocol_v1.json')
    seeds=list(range(seed_offset,seed_offset+trials))
    if any(seed not in P['budget']['seeds'] for seed in seeds):raise ValueError('seed outside frozen protocol budget')
    case=c.case_dir(case_id);folder=case/c.STAGES[4]/'runs'/run_id
    if folder.exists():raise ValueError('不覆盖物理结果，请使用新 run-id')
    folder.mkdir(parents=True);c.write(folder/'frozen_inputs.json',{'case_id':case_id,'construction_run_id':construction_run_id,'construction_manifest_sha256':c.digest(construction/'evidence_manifest.json'),'protocol_sha256':config['protocol_sha256'],'trials_requested':trials,'execution_seeds':seeds,'config':config,'expectation':config['config']['expectation'],'started_at_utc':c.now(),'source_code_sha256':{str(p.relative_to(c.WORKSPACE)):c.digest(p) for p in [c.WORKSPACE/'src/lastmile/navigation.py',c.WORKSPACE/'src/lastmile/calibration_runner.py',c.WORKSPACE/'src/lastmile/simulation.py',c.WORKSPACE/'src/lastmile/validation.py',c.WORKSPACE/'src/lastmile/workflow/closed_loop.py']},'navigation_semantics':'position servo actions only; no qpos writes or restore between initial state and pick'})
    from ..simulation import Simulation,body_pose
    from ..navigation import execute_navigation
    from ..calibration_runner import run_one
    from ..validation import validate_motion_prefix,validate_trace
    import mujoco
    sim=Simulation.from_cached(case/'inputs/engine_context',14,case/c.STAGES[0],folder/'engine');source_qpos=sim.data.qpos.copy();source_qvel=sim.data.qvel.copy()
    sim.initial_state=np.load(construction/'case_initial_snapshot.npz')['state'].copy();sim.restore();snapshot_base_only(sim,source_qpos,source_qvel)
    base_episode=copy.deepcopy(sim.episode);original_restore=sim.restore;P=c.read(c.WORKSPACE/'configs/protocols/protocol_v1.json');results=[]
    c.update(case_id,c.STAGES[4],status='running',run_id=run_id,closed_loop_verified=False)
    for trial in range(trials):
        dest=folder/f'trial_{trial:02d}';dest.mkdir();sim.run_dir=dest;sim.restore=original_restore;sim.restore();sim.episode=copy.deepcopy(base_episode);sim.tick_observer=None
        nav_rows=[];states=[{'time_s':float(sim.data.time),'phase':'start','qpos':sim.data.qpos.copy()}]
        initial={'time_s':float(sim.data.time),'base':sim.group('base').tolist(),'head':sim.group('head').tolist(),'left_arm':sim.group('left_arm').tolist(),'right_arm':sim.group('right_arm').tolist(),'target_pose':body_pose(sim.data,sim.target).tolist()}
        def forbid_restore():raise RuntimeError('禁止导航之后恢复/重置初始快照')
        sim.restore=forbid_restore
        try:
            staging=np.array(config['staging_base']);goal=np.array(config['goal_base']);speed=config['config']['navigation']['max_speed_m_s'];stride=config['config']['navigation']['state_record_stride']
            first=execute_navigation(sim,np.array(config['navigation_segments'][0]),staging,nav_rows,states,max_speed=speed,record_stride=stride)
            n=len(nav_rows);sn=len(states)
            arrival=execute_navigation(sim,np.array(config['navigation_segments'][1]),goal,nav_rows,states,max_speed=speed,record_stride=stride)
            for row in nav_rows[n:]:
                if row['phase']=='start':row['phase']='navigation'
            for state in states[sn:]:
                if state['phase']=='start':state['phase']='navigation'
            failure=validate_navigation_rows(nav_rows,initial,P['thresholds'])
            if failure:raise RuntimeError(failure)
            arrival_state=np.zeros_like(sim.initial_state);mujoco.mj_getState(sim.model,sim.data,arrival_state,sim.state_spec);np.savez_compressed(dest/'actual_arrival_snapshot.npz',state=arrival_state)
            # 只更新验收参考为实测底盘。此处不写任何 qpos、qvel 或对象位姿。
            measured=sim.group('base').copy();sim.episode['robot']['init_qpos']['base']=measured.tolist();arrival_time=float(sim.data.time);counter=[0]
            def observer(s,phase):
                counter[0]+=1
                if counter[0]%stride==0:states.append({'time_s':float(s.data.time),'phase':phase,'qpos':s.data.qpos.copy()})
            sim.tick_observer=observer
            physics=config['config']['physics'];pick=run_one(case/'inputs/engine_context',14,physics['side'],physics['h'],seeds[trial],dest/'pick',1,sim=sim,world_geometry='mesh',time_dilation=physics['time_dilation'],approach_overshoot=physics['approach_overshoot_m'],grasp_ids=[physics['grasp_id']],restore_start=False)
            prefix=c.read(dest/'pick/torso_adjust_trace.json');prefix_failure=validate_motion_prefix(prefix['samples'],prefix['initial'],P['thresholds']);trace_path=dest/f'pick/grasp_{physics["grasp_id"]:04d}_trace.json'
            verdict={'status':pick['status'],'reason':pick.get('reason')}
            if trace_path.exists():
                trace=c.read(trace_path);verdict=validate_trace(trace['samples'],trace['initial'],P['thresholds'])
            if prefix_failure:verdict={'status':'executed_failure','reason':prefix_failure}
            if abs(prefix['samples'][0]['time_s']-(arrival_time+.004))>1e-8:raise RuntimeError('arrival_to_pick_time_discontinuity')
            if not np.array_equal(prefix['initial']['base'],measured.tolist()):raise RuntimeError('arrival_to_pick_base_reference_changed')
            times=np.array([s['time_s'] for s in states])
            if np.any(np.diff(times)<=0):raise RuntimeError('video_state_time_reset')
            result={'trial':trial,'seed':seeds[trial],**verdict,'pick_backend_result':pick,'actual_arrival':arrival,'staging_arrival':first,'arrival_used_without_reset':True,'restore_calls_during_navigation_and_pick':0,'navigation_samples':len(nav_rows),'navigation_validation':'passed','navigation_end_time_s':arrival_time,'pick_start_base':measured.tolist(),'total_physical_time_s':float(sim.data.time),'start_evidence':'conservative_both_arm_all_yaw_continuous_torso_contact_bound','start_pose_id':config['start_pose_id'],'construction_manifest_sha256':c.digest(construction/'evidence_manifest.json'),'new_physical_execution':True}
        except Exception as exc:
            traceback.print_exc();result={'trial':trial,'seed':seeds[trial],'status':'executed_failure' if isinstance(exc,RuntimeError) and not 'CUDA' in str(exc) else 'infrastructure_error','reason':str(exc),'partial_navigation_samples':len(nav_rows),'new_physical_execution':True}
        finally:
            sim.tick_observer=None;sim.restore=original_restore
            c.write(dest/'navigation_trace.json',{'initial':initial,'samples':nav_rows,'complete':result.get('navigation_validation')=='passed'})
            np.savez_compressed(dest/'video_states.npz',qpos=np.array([r['qpos'] for r in states]),time_s=np.array([r['time_s'] for r in states]),phase=np.array([r['phase'] for r in states]))
            c.write(dest/'result.json',result);results.append(result)
            summary={'case_id':case_id,'run_id':run_id,'construction_run_id':construction_run_id,'trials_requested':trials,'trials_completed':len(results),'strict_successes':sum(r['status']=='verified_success' for r in results),'closed_loop_verified':len(results)==trials and all(r['status']=='verified_success' for r in results),'repeat_acceptance_verified':len(results)==config['config']['physics']['trials'] and all(r['status']=='verified_success' for r in results),'human_final_accepted':False,'trials':results}
            c.write(folder/'summary.json',summary);print('TRIAL',trial,result['status'],result.get('reason'),flush=True)
        if result['status']!='verified_success':break
    files={}
    for path in folder.rglob('*'):
        if path.is_file() and path.name not in ['evidence_manifest.json','compiled_model.mjb']:files[str(path.relative_to(folder))]=c.digest(path)
    c.write(folder/'evidence_manifest.json',{'case_id':case_id,'run_id':run_id,'files':files})
    c.write(case/c.STAGES[4]/'latest.json',{'run_id':run_id,'construction_run_id':construction_run_id})
    c.update(case_id,c.STAGES[4],status='closed_loop_verified_pending_human' if summary['repeat_acceptance_verified'] else 'pilot_verified' if summary['closed_loop_verified'] else 'failed',run_id=run_id,closed_loop_verified=summary['closed_loop_verified'],repeat_acceptance_verified=summary['repeat_acceptance_verified'],strict_successes=summary['strict_successes'],human_final_accepted=False)
    return summary

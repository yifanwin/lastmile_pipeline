"""闭环离线审计：重验原始轨迹，检查构造/来源及无到站重置证据。"""
import numpy as np
from . import core as c
from .construction import checked_file
from .hard_start import require_robot_candidate,snapshot_base_only
from .closed_loop import validate_navigation_rows

def audit_closed_loop(case_id,run_id):
    c.case_dir(run_id);case=c.case_dir(case_id);folder=case/c.STAGES[4]/'runs'/run_id;summary=c.read(folder/'summary.json');packet=c.read(folder/'evidence_manifest.json')
    for name,sha in packet['files'].items():checked_file(folder,name,sha)
    frozen=c.read(folder/'frozen_inputs.json');construction,config=require_robot_candidate(case_id,frozen['construction_run_id'])
    if c.digest(construction/'evidence_manifest.json')!=frozen['construction_manifest_sha256']:raise ValueError('构造证据版本已变化')
    from ..simulation import Simulation
    from ..validation import validate_trace,validate_motion_prefix
    from .restore import physical_signature
    import mujoco
    sim=Simulation.from_cached(case/'inputs/engine_context',14,case/c.STAGES[0],folder/'audit_engine');source_qpos=sim.data.qpos.copy();source_qvel=sim.data.qvel.copy();P=c.read(c.WORKSPACE/'configs/protocols/protocol_v1.json')
    mujoco.mj_setState(sim.model,sim.data,np.load(construction/'case_initial_snapshot.npz')['state'],sim.state_spec);mujoco.mj_forward(sim.model,sim.data)
    placement=snapshot_base_only(sim,source_qpos,source_qvel)
    if not np.array_equal(sim.group('base'),np.array(config['start_base'])):raise ValueError('case initial base mismatch')
    if physical_signature(sim.model)!=config['physical_model_signature_sha256']:raise ValueError('physical model mismatch')
    checks=[]
    for record in summary['trials']:
        trial=folder/f'trial_{record["trial"]:02d}';nav=c.read(trial/'navigation_trace.json')
        for row in nav['samples']:
            for key in ['time_s','base','torso','head','selected_arm_q','idle_arm','target_pose']:
                if not np.isfinite(np.asarray(row[key],float)).all():raise ValueError('导航轨迹出现非有限数据')
        failure=validate_navigation_rows(nav['samples'],nav['initial'],P['thresholds'])
        if failure:raise ValueError('导航审计未通过：'+failure)
        prefix=c.read(trial/'pick/torso_adjust_trace.json');trace=c.read(trial/'pick/grasp_0794_trace.json');prefix_failure=validate_motion_prefix(prefix['samples'],prefix['initial'],P['thresholds']);verdict=validate_trace(trace['samples'],trace['initial'],P['thresholds'])
        if prefix_failure or verdict['status']!='verified_success' or record['status']!=verdict['status']:raise ValueError('严格 Pick 原始轨迹未通过/标签不一致')
        mujoco.mj_setState(sim.model,sim.data,np.load(trial/'actual_arrival_snapshot.npz')['state'],sim.state_spec);mujoco.mj_forward(sim.model,sim.data)
        if not np.array_equal(sim.group('base'),np.array(prefix['initial']['base'])):raise ValueError('到站状态与 Pick 前缀初始状态不一致')
        if abs(sim.data.time-nav['samples'][-1]['time_s'])>1e-9 or abs(prefix['samples'][0]['time_s']-sim.data.time-.004)>1e-8:raise ValueError('到站/抓取时间断裂')
        if not np.array_equal(nav['samples'][-1]['base'],prefix['initial']['base']):raise ValueError('底盘到站重置')
        z=np.load(trial/'video_states.npz')
        if np.any(np.diff(z['time_s'])<=0):raise ValueError('保存状态时间回退')
        if not np.array_equal(z['qpos'][0],np.load(construction/'case_initial_snapshot.npz')['state'][1:1+sim.model.nq]):
            # INTEGRATION state begins with time and qpos (MuJoCo explicit component size).
            raise ValueError('视频初始状态未取自派生快照')
        checks.append({'trial':record['trial'],'navigation_samples':len(nav['samples']),'navigation_revalidated':True,'pick_revalidated':verdict,'arrival_snapshot_base_exactly_prefix':True,'arrival_to_pick_first_tick_gap_s':.004,'saved_state_time_monotonic':True})
    audit={'case_id':case_id,'run_id':run_id,'all_checks_passed':True,'placement_audit':placement,'trials':checks,'strict_successes_recomputed':len(checks),'repeat_acceptance_verified':len(checks)==config['config']['physics']['trials'],'initial_and_arrival_snapshots_checked':True}
    c.write(folder/'evidence_audit.json',audit)
    # 新增审计结果，不修改任何原始轨迹或试验标签。
    packet['files']['evidence_audit.json']=c.digest(folder/'evidence_audit.json');c.write(folder/'evidence_manifest.json',packet)
    return audit

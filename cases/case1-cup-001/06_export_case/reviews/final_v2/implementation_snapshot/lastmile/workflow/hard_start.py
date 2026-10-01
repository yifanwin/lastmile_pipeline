"""步骤 4 robot_only：从站位图选择有强失败证据的困难起点。"""
import copy
from pathlib import Path
import numpy as np
from . import core as c
from .construction import require_station_evidence,checked_file
from .station_geometry import place,initial_collision,reach_bound,wrap_yaw

def snapshot_base_only(sim,source_qpos,source_qvel):
    indices=[sim.qpos_adr[n] for n in ['base_x','base_y','base_theta']]
    keep=np.ones(sim.model.nq,bool);keep[indices]=False
    if not np.array_equal(sim.data.qpos[keep],source_qpos[keep]) or not np.array_equal(sim.data.qvel,source_qvel):raise ValueError('派生起点改变了非底盘状态')
    return {'only_base_qpos_changed':True,'non_base_qpos_exactly_source':True,'all_qvel_exactly_source':True}

def turn_is_clear(sim,xy,begin,end):
    delta=wrap_yaw(end-begin)
    for theta in np.linspace(begin,begin+delta,max(2,int(abs(delta)/.02)+1)):
        place(sim,[*xy,theta])
        if initial_collision(sim):return False
    return True

def construct_robot_start(case_id,run_id='robot_move_v1'):
    c.case_dir(run_id);station=require_station_evidence(case_id,'coarse_v1');config=c.load_config('04_robot_start.yaml')
    if config['method']!='robot_only' or config['allowed_scene_edits']!=['robot_initial_base'] or any(config[k] for k in ['allow_target_pose_change','allow_furniture_change','allow_asset_replacement']):raise ValueError('禁止未授权场景修改')
    case=c.case_dir(case_id);folder=case/c.STAGES[3]/'runs'/run_id
    if folder.exists():raise ValueError('不覆盖构造证据，请使用新的 run-id')
    folder.mkdir(parents=True)
    c.write(folder/'human_choice.json',{'case_id':case_id,'actor':'human','decision':'method_selected','method':'robot_only','allow_scene_edits':['robot_initial_base'],'source':'explicit_user_chat_request','user_answer':'继续构建真实困难起点case，这个点就是 步骤 3 中的站位图中执行失败或者超出范围的点，它们的完成可以先用A star算法到达目标附近，然后再执行curobo抓取。','supersedes_method_only':'original_only','candidate_approval':False,'captured_at_utc':c.now()})
    from ..simulation import Simulation,body_pose
    from ..navigation import build_configuration_grid
    from .restore import physical_signature,render_views
    sim=Simulation.from_cached(case/'inputs/engine_context',14,case/c.STAGES[0],folder/'engine');facts=c.read(case/c.STAGES[0]/'scene_facts.json')
    if physical_signature(sim.model)!=facts['physical_model_signature_sha256']:raise ValueError('model signature mismatch')
    source_qpos=sim.data.qpos.copy();source_qvel=sim.data.qvel.copy();rows=c.read(station/'candidates.json');lookup={r['pose_id']:r for r in rows};goal=np.array(lookup['A']['base']);center=np.array(facts['support_AABB_center']);size=np.array(facts['support_AABB_size']);bound=reach_bound(sim)
    if abs(bound['contact_radius_m']-c.read(station/'reach_bound.json')['contact_radius_m'])>1e-9:raise ValueError('工作空间界已改变')
    diagnostics=[];selected=None
    for pose_id in config['candidate_priority']:
        row=lookup[pose_id];start=np.array(row['base']);place(sim,start)
        if row['geometry_status']!='outside_contact_bound' or initial_collision(sim):raise ValueError('候选不满足强距离证据或初始碰撞检查')
        direction=goal[:2]-center[:2];direction/=np.linalg.norm(direction)
        for offset in config['navigation']['staging_offsets_m']:
            staging=np.r_[goal[:2]+direction*offset,goal[2]];trial=folder/'geometry'/f'{pose_id}_staging{offset:.2f}';sim.run_dir=trial;trial.mkdir(parents=True)
            try:
                place(sim,start);g1=build_configuration_grid(sim,start,staging,center,size,config['navigation']['grid_spacing_m'],config['navigation']['configuration_margin_m'])
                if not turn_is_clear(sim,staging[:2],start[2],goal[2]):raise ValueError('转向点 swept collision')
                sim.run_dir=trial/'final_approach';sim.run_dir.mkdir();place(sim,staging);g2=build_configuration_grid(sim,staging,goal,center,size,config['navigation']['grid_spacing_m'],config['navigation']['configuration_margin_m'])
                place(sim,start)
                try:counts=render_views(sim,trial,center,size)
                except Exception as exc:raise RuntimeError('起点可视性渲染基础设施失败') from exc
                if counts['initial_robot_view']<8:raise ValueError('起点 head_camera 目标不可见')
                selected=(row,staging,g1,g2,trial,counts);diagnostics.append({'pose_id':pose_id,'offset_m':offset,'accepted_geometry':True,'target_visible_pixels':counts['initial_robot_view']});break
            except ValueError as exc:
                diagnostics.append({'pose_id':pose_id,'offset_m':offset,'accepted_geometry':False,'reason':str(exc)});print('candidate rejected',diagnostics[-1],flush=True)
        if selected:break
    c.write(folder/'selection_diagnostics.json',diagnostics)
    if selected is None:raise ValueError('站位图候选无保守路线/转向/可视性通过；不扩大场景修改')
    row,staging,g1,g2,trial,counts=selected;place(sim,row['base']);sim.run_dir=folder/'engine'
    audit=snapshot_base_only(sim,source_qpos,source_qvel)
    import mujoco
    state=np.zeros_like(sim.initial_state);mujoco.mj_getState(sim.model,sim.data,state,sim.state_spec);np.savez_compressed(folder/'case_initial_snapshot.npz',state=state)
    dist=float(np.linalg.norm(np.array(row['base'][:2])-sim.data.xpos[sim.target,:2]));margin=dist-bound['contact_radius_m']
    if margin<=0:raise ValueError('起点未超出强工作空间界')
    certificate={**bound,'pose_id':row['pose_id'],'start_base':row['base'],'target_xyz':sim.data.xpos[sim.target].tolist(),'start_base_to_target_xy_m':dist,'unreachable_margin_m':margin,'claim':'no_bilateral_finger_contact_possible_at_this_xy_under_protocol_for_either_arm_all_yaw_and_continuous_legal_torso','same_side_unreachable_claim':False,'full_fixed_base_pick_attempt_executed':False,'not_budget_exhaustion_label':True}
    c.write(folder/'reach_certificate.json',certificate)
    final={'case_id':case_id,'construction_run_id':run_id,'source_index':14,'method':'robot_only','case_type':'Case 1-R','start_pose_id':row['pose_id'],'start_side':row['side'],'start_base':row['base'],'goal_pose_id':'A','goal_base':goal.tolist(),'staging_base':staging.tolist(),'navigation_segments':[g1['path'].tolist(),g2['path'].tolist()],'table_center':center.tolist(),'table_size':size.tolist(),'target_name':sim.target_name,'target_pose':body_pose(sim.data,sim.target).tolist(),'target_visible_pixels':counts['initial_robot_view'],'views_dir':str((trial/'views').relative_to(folder)),'scene_edits':[{'field':'robot.initial_base','from':facts['initial_base'],'to':row['base']}],'placement_audit':audit,'physical_model_signature_sha256':physical_signature(sim.model),'source_snapshot_sha256':c.digest(case/c.STAGES[0]/'initial_snapshot.npz'),'review_packet_sha256':c.review_packet_valid(case_id),'station_manifest_sha256':c.digest(station/'evidence_manifest.json'),'config':config,'policy_config_sha256':c.digest(c.WORKSPACE/'configs/04_robot_start.yaml'),'protocol_sha256':c.digest(c.WORKSPACE/'configs/protocols/protocol_v1.json'),'status':'geometry_candidate_ready_not_closed_loop','navigation_verified':False,'case1_goal_achieved':False}
    c.write(folder/'case_config.json',final)
    names=['case_config.json','case_initial_snapshot.npz','reach_certificate.json','human_choice.json','selection_diagnostics.json']
    for path in (folder/'geometry').rglob('*'):
        if path.is_file():names.append(str(path.relative_to(folder)))
    c.write(folder/'evidence_manifest.json',{'case_id':case_id,'run_id':run_id,'files':{name:c.digest(folder/name) for name in names}})
    c.write(case/c.STAGES[3]/'latest.json',{'run_id':run_id,'method':'robot_only','supersedes_original_only_screening':True})
    c.update(case_id,c.STAGES[3],status='geometry_candidate_ready',method='robot_only',run_id=run_id,human_method_selected=True,candidate_selected=True,can_enter_step5=True,scene_edits=['robot_initial_base'],screening_complete=True,original_only_result_preserved=True)
    print('Selected',row['pose_id'],'distance',dist,'bound',bound['contact_radius_m'],flush=True)
    return final

def require_robot_candidate(case_id,run_id):
    c.case_dir(run_id);station=require_station_evidence(case_id,'coarse_v1');folder=c.case_dir(case_id)/c.STAGES[3]/'runs'/run_id;packet=c.read(folder/'evidence_manifest.json')
    if packet['case_id']!=case_id or packet['run_id']!=run_id:raise ValueError('构造包来源错配')
    for name,sha in packet['files'].items():checked_file(folder,name,sha)
    config=c.read(folder/'case_config.json');choice=c.read(folder/'human_choice.json')
    if choice.get('actor')!='human' or choice.get('method')!='robot_only' or choice.get('allow_scene_edits')!=['robot_initial_base']:raise ValueError('无有效人工 robot_only 选择')
    for path,key in [(station/'evidence_manifest.json','station_manifest_sha256'),(c.WORKSPACE/'configs/04_robot_start.yaml','policy_config_sha256'),(c.WORKSPACE/'configs/protocols/protocol_v1.json','protocol_sha256')]:
        if c.digest(path)!=config[key]:raise ValueError('上游协议/证据已变化')
    if c.review_packet_valid(case_id)!=config['review_packet_sha256']:raise ValueError('恢复审阅包已变化')
    cert=c.read(folder/'reach_certificate.json')
    if cert['unreachable_margin_m']<=0 or config['target_visible_pixels']<8 or not config['placement_audit']['only_base_qpos_changed']:raise ValueError('困难起点几何证据不足')
    return folder,config

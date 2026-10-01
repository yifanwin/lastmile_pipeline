"""步骤 3：冻结场景、几何筛选、双臂五高度规划、关键站位动态试验。"""
import copy,gc,json
import numpy as np
from .core import *
from .station_geometry import candidates,floor_mask,place,initial_collision,reach_bound

def build_planner(sim,side,h,world,folder,config):
 import yaml
 from molmo_spaces.planner.curobo_planner import CuroboPlanner,CuroboPlannerConfig
 from ..protocol import arm_only_config
 root=sim.assets/'robots/rby1m/curobo_config';template=yaml.safe_load((root/'rby1m_holobase.yml').read_text());idle='right' if side=='left' else 'left'
 cfg=arm_only_config(template,side,h,sim.group(idle+'_arm'),sim.group('head'));kin=cfg['robot_cfg']['kinematics'];kin.update(urdf_path=str(root/'urdf/model_holobase.urdf'),asset_root_path=str(root/'urdf/meshes'),collision_spheres=str(root/'rby1m_holobase_spheres.yml'),usd_robot_root=str(root))
 path=folder/f'{side}_h{h:.4f}.yaml';path.write_text(yaml.safe_dump(cfg,sort_keys=False));p=config['planning']
 planner=CuroboPlanner(CuroboPlannerConfig(curobo_robot_config_path=str(path),world_config=world,collision_cache={'mesh':3,'obb':512},num_ik_seeds=p['ik_seeds'],num_trajopt_seeds=p['trajopt_seeds'],max_attempts=p['max_attempts'],collision_activation_distance=p['activation_distance_m'],time_dilation_factor=config['physics']['time_dilation']))
 if planner.joint_names!=[f'{side}_arm_{i}' for i in range(7)]:raise ValueError('planner released forbidden joints')
 return planner

def plan_pose(sim,planner,side,h,reference_base,world_grasps,row,config):
 from ..simulation import body_pose,pose7
 from ..protocol import to_base_frame,successful_plan
 base=body_pose(sim.data,sim.model.body('robot_0/base').id);frame=np.linalg.inv(base)@reference_base
 planner.update_world_obstacle_poses({'actual_collision_geoms':pose7(frame)})
 # 独立检查真实 FK；转移的是世界障碍刚体坐标，不移场景对象。
 fk=np.asarray(planner.fk_solve(sim.group(side+'_arm').tolist()));actual=to_base_frame(sim.tcp(side),base)
 if np.linalg.norm(fk[:3]-actual[:3,3])>.001:raise ValueError('station FK frame mismatch')
 q=sim.group(side+'_arm').tolist();gid=config['planning']['grasp_ids'][0];grasp=world_grasps[gid].copy()
 out={'pose_id':row['pose_id'],'side':side,'h':h,'grasp_id':gid,'status':'search_exhausted','ik_solved':False,'paths':[],'planning_only':True}
 mask=planner.ik_solve_batch([pose7(to_base_frame(grasp,base))],seed_config=q,num_seeds=config['planning']['ik_seeds'],disable_collision=False,pad_to=1)
 if not mask[0]:out['reason']='collision_aware_ik_budget_exhausted';return out
 out['ik_solved']=True;pre=grasp.copy();pre[:3,3]-=grasp[:3,2]*.08;grasp[:3,3]+=grasp[:3,2]*config['physics']['approach_overshoot_m'];lift=grasp.copy();lift[2,3]+=.1
 for phase,goal in [('pregrasp',pre),('approach',grasp),('lift',lift)]:
  path,result=planner.motion_plan(q,[pose7(to_base_frame(goal,base))]);ok=successful_plan(result,path)
  out['paths'].append({'phase':phase,'success':ok,'status':str(result.status),'waypoints':len(path) if ok else 0})
  if not ok:out['reason']=phase+'_plan_budget_exhausted';return out
  q=path[-1]
 out.update(status='full_path_planned',reason=None);return out

def select_physics(rows,planning,budget):
 # 不从图像填写坐标。优先已完成三个路径的离散候选；同一 xy 先覆盖空间，再补朝向/高度。
 paths=[p for p in planning if p['status']=='full_path_planned'];by_id={r['pose_id']:r for r in rows};chosen=[]
 A=[p for p in paths if p['pose_id']=='A' and p['side']=='left' and abs(p['h']-.738)<1e-6]
 if A:chosen.append(A[0])
 else:chosen.append({'pose_id':'A','side':'left','h':.738,'grasp_id':794,'status':'A_control_even_if_prefilter_failed'})
 remaining=[p for p in paths if not any(p['pose_id']==c['pose_id'] and p['side']==c['side'] and p['h']==c['h'] for c in chosen)]
 while remaining and len(chosen)<budget-2:
  # 最远点覆盖；偏向 h=.738 / 左臂的已稳定协议，距已有站位相同则按冻结顺序打破平局。
  def priority(p):
   xy=np.array(by_id[p['pose_id']]['base'][:2]);d=min(np.linalg.norm(xy-np.array(by_id[c['pose_id']]['base'][:2])) for c in chosen)
   return (round(float(d),6),p['h'],p['side']=='left',-int(p['pose_id'][1:]) if p['pose_id']!='A' else 0)
  p=max(remaining,key=priority);chosen.append(p);remaining=[x for x in remaining if x['pose_id']!=p['pose_id']]
 # 两个搜索边界诊断：只跑同一稳定抓法，不把预算耗尽当成真实二值失败。
 seen={p['pose_id'] for p in chosen};eligible=[r for r in rows if r['geometry_status']=='eligible' and r['pose_id'] not in seen]
 for r in sorted(eligible,key=lambda r:r['target_distance_m'],reverse=True)[:max(0,budget-len(chosen))]:
  chosen.append({'pose_id':r['pose_id'],'side':'left','h':.738,'grasp_id':794,'status':'boundary_budget_check'})
 return chosen[:budget]

def run_station_map(case_id,run_id='coarse_v1',resume=False):
 require_next_stage(case_id);human=require_review(case_id);config=load_config('03_station_map.yaml')
 if config['planning']['grasp_ids']!=[794]:raise ValueError('初版仅允许已冻结稳定 grasp 794')
 if not all(0<=h<=.738 for h in config['planning']['heights']):raise ValueError('height outside calibrated protocol')
 # run_id 使用相同的安全标识规则。
 case_dir(run_id);stage=case_dir(case_id)/STAGES[2];folder=stage/'runs'/run_id;folder.mkdir(parents=True,exist_ok=True)
 frozen=folder/'frozen_inputs.json';sha=digest(WORKSPACE/'configs/03_station_map.yaml');hsha=digest(case_dir(case_id)/STAGES[0]/'human_review.json')
 if frozen.exists():
  old=read(frozen)
  if not resume:raise ValueError('运行已存在，使用 --resume 或新 --run-id')
  if old['config_sha256']!=sha or old['human_review_sha256']!=hsha or old['review_packet_sha256']!=review_packet_valid(case_id):raise ValueError('配置/审批/场景已改变，不允许续跑')
 else:
  write(frozen,{'case_id':case_id,'run_id':run_id,'config':config,'config_sha256':sha,'human_review_sha256':hsha,'review_packet_sha256':review_packet_valid(case_id),'protocol_sha256':digest(WORKSPACE/'configs/protocols/protocol_v1.json'),'created_at_utc':now(),'reference_document':'MoMaKitchen/MoMa-Kitchen_数据生成流程解析.md','reference_sha256':digest(WORKSPACE.parent/'MoMaKitchen/MoMa-Kitchen_数据生成流程解析.md'),'placement_semantics':'restore original full-state then set only base pose for independent manipulation oracle; NOT navigation','expectation':config['expectation']})
 update(case_id,STAGES[2],status='running',run_id=run_id)
 import mujoco,torch
 from scipy.ndimage import distance_transform_edt
 from ..simulation import Simulation,body_pose
 from ..calibration_runner import mesh_world_obstacles,run_one
 from ..validation import validate_trace,validate_motion_prefix
 from .restore import physical_signature
 context=case_dir(case_id)/'inputs/engine_context';sim=Simulation.from_cached(context,14,case_dir(case_id)/STAGES[0],folder/'engine');facts=read(case_dir(case_id)/STAGES[0]/'scene_facts.json')
 if physical_signature(sim.model)!=facts['physical_model_signature_sha256']:raise ValueError('model signature mismatch')
 A=np.array(facts['initial_base']);center=np.array(facts['support_AABB_center']);size=np.array(facts['support_AABB_size']);target=sim.data.xpos[sim.target].copy();original_episode=copy.deepcopy(sim.episode)
 if not (folder/'candidates.json').exists():
  bound=reach_bound(sim);write(folder/'reach_bound.json',bound)
  s=config['sampling'];low=center[:2]-size[:2]/2-1.1;high=center[:2]+size[:2]/2+1.1;spacing=s['floor_grid_spacing_m'];xs=np.arange(low[0],high[0]+spacing,spacing);ys=np.arange(low[1],high[1]+spacing,spacing);floors=floor_mask(sim,xs,ys)
  # 外侧 padding 避免 EDT 在网格边缘错误扩张。
  clearance=distance_transform_edt(np.pad(floors,1))[1:-1,1:-1]*spacing
  rows=candidates(center,size,target,A,human['suitable_sides'],s)
  for r in rows:
   base=np.array(r['base']);ix=np.rint((base[:2]-low)/spacing).astype(int);covered=0<=ix[0]<len(xs) and 0<=ix[1]<len(ys) and clearance[tuple(ix)]>=s['floor_footprint_radius_m']
   place(sim,base);bad=initial_collision(sim);dist=float(np.linalg.norm(base[:2]-target[:2]));r.update(target_distance_m=dist,initial_collision=bad,floor_footprint_covered=bool(covered))
   r['geometry_status']='floor_footprint_rejected' if not covered else 'initial_pose_collision' if bad else 'outside_contact_bound' if dist>bound['contact_radius_m'] else 'eligible'
   r['planning_status']='not_run';r['physics_status']='not_run'
  np.savez_compressed(folder/'floor_grid.npz',xs=xs,ys=ys,floors=floors,clearance=clearance);write(folder/'candidates.json',rows)
  print('geometry',dict(__import__('collections').Counter(r['geometry_status'] for r in rows)),flush=True)
 else:rows=read(folder/'candidates.json')
 eligible=[r for r in rows if r['geometry_status']=='eligible'];place(sim,A);reference_base=body_pose(sim.data,sim.model.body('robot_0/base').id).copy();world=mesh_world_obstacles(sim,max_distance=config['planning']['mesh_roi_distance_m'])
 if max(np.linalg.norm(np.array(r['base'][:2])-A[:2]) for r in rows)+1.3>config['planning']['mesh_roi_distance_m']:raise ValueError('world mesh ROI cannot cover sampled stations plus arm extent')
 asset=read(case_dir(case_id)/'inputs/asset.json')
 if digest(asset['asset_xml'])!=asset['asset_xml_sha256'] or digest(asset['grasp_path'])!=asset['grasp_sha256']:raise ValueError('asset/grasp source changed')
 with np.load(asset['grasp_path']) as z:grasps=z['transforms'].astype(float)
 u,_,v=np.linalg.svd(grasps[:,:3,:3]);grasps[:,:3,:3]=u@v;world_grasps=body_pose(sim.data,sim.target)@grasps
 planning=read(folder/'planning.json') if (folder/'planning.json').exists() else [];done={(p['pose_id'],p['side'],p['h']) for p in planning}
 for side in config['planning']['arms']:
  for h in config['planning']['heights']:
   todo=[r for r in eligible if (r['pose_id'],side,h) not in done]
   if not todo:continue
   place(sim,A,h);planner=build_planner(sim,side,h,world,folder,config)
   try:
    for n,r in enumerate(todo):
     place(sim,r['base'],h);torch.manual_seed(config['planning']['seed']);np.random.seed(config['planning']['seed']);bad=initial_collision(sim)
     if bad:p={'pose_id':r['pose_id'],'side':side,'h':h,'grasp_id':794,'status':'height_pose_collision','ik_solved':False,'planning_only':True,'collisions':bad}
     else:p=plan_pose(sim,planner,side,h,reference_base,world_grasps,r,config)
     planning.append(p);write(folder/'planning.json',planning)
     print('planning',side,h,n+1,'/',len(todo),r['pose_id'],p['status'],flush=True)
   finally:del planner;gc.collect();torch.cuda.empty_cache()
 selection_path=folder/'physics_selection.json'
 if selection_path.exists():selection=read(selection_path)
 else:selection=select_physics(rows,planning,config['physics']['max_trials']);write(selection_path,selection)
 physical=read(folder/'physics.json') if (folder/'physics.json').exists() else [];finished={p['trial_id'] for p in physical};row_by_id={r['pose_id']:r for r in rows};P=read(WORKSPACE/'configs/protocols/protocol_v1.json')
 for i,choice in enumerate(selection):
  trial_id=f'T{i:02d}_{choice["pose_id"]}_{choice["side"]}_h{choice["h"]:.4f}'
  if trial_id in finished:continue
  run=folder/'trials'/trial_id;run.mkdir(parents=True,exist_ok=True);sim.run_dir=run;place(sim,row_by_id[choice['pose_id']]['base']);sim.episode=copy.deepcopy(original_episode);sim.episode['robot']['init_qpos']['base']=row_by_id[choice['pose_id']]['base']
  state=np.zeros_like(sim.initial_state);mujoco.mj_getState(sim.model,sim.data,state,sim.state_spec);np.savez_compressed(run/'placed_snapshot.npz',state=state)
  write(run/'placement.json',{'only_robot_base_changed':True,'full_original_snapshot_restored_first':True,'source_snapshot_sha256':digest(case_dir(case_id)/STAGES[0]/'initial_snapshot.npz'),'candidate_base':row_by_id[choice['pose_id']]['base'],'target_world_pose_before':body_pose(sim.data,sim.target).tolist(),'planning_choice':choice,'not_navigation_success':True})
  result=run_one(context,14,choice['side'],choice['h'],config['planning']['seed'],run,1,sim=sim,world_geometry='mesh',time_dilation=config['physics']['time_dilation'],approach_overshoot=config['physics']['approach_overshoot_m'],grasp_ids=[794],restore_start=False)
  trace_path=run/'grasp_0794_trace.json';prefix_path=run/'torso_adjust_trace.json';label=None
  if trace_path.exists():
   t=read(trace_path);p=read(prefix_path);verdict=validate_trace(t['samples'],t['initial'],P['thresholds']);prefix_failure=validate_motion_prefix(p['samples'],p['initial'],P['thresholds'])
   if prefix_failure:verdict={'status':'executed_failure','reason':prefix_failure}
   if verdict['status']!='infrastructure_error':label=int(verdict['status']=='verified_success')
  else:verdict={'status':result['status'],'reason':result.get('reason','planner did not execute complete Pick')}
  record={'trial_id':trial_id,'pose_id':choice['pose_id'],'base':row_by_id[choice['pose_id']]['base'],'side':choice['side'],'h':choice['h'],'grasp_id':794,'result':result,'revalidated_verdict':verdict,'binary_pick_label':label,'label_is_complete_dynamic_pick':label is not None,'result_path':str((run/'result.json').relative_to(folder)),'snapshot_restored':True,'single_trial_not_success_rate':True}
  if trace_path.exists():record.update(trace_path=str(trace_path.relative_to(folder)),trace_sha256=digest(trace_path))
  physical.append(record);write(folder/'physics.json',physical);print('physics',trial_id,verdict,'binary',label,flush=True)
 sim.episode=original_episode
 from .station_plot import export_map
 result=export_map(case_id,run_id)
 from .station_audit import audit_station_map
 audit_station_map(case_id,run_id)
 update(case_id,STAGES[2],status='coarse_map_complete',run_id=run_id,map_ready=True,summary_path=str((folder/'summary.json').relative_to(WORKSPACE)),full_navigation_verified=False)
 return result

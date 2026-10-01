"""站位图的原始证据复核与哈希清单，不重复生成成功标签。"""
import numpy as np
from .core import *
def audit_station_map(case_id,run_id='coarse_v1'):
 import mujoco
 from ..validation import validate_trace,validate_motion_prefix
 folder=case_dir(case_id)/STAGES[2]/'runs'/run_id
 asset=read(case_dir(case_id)/'inputs/asset.json')
 if digest(asset['asset_xml'])!=asset['asset_xml_sha256'] or digest(asset['grasp_path'])!=asset['grasp_sha256']:raise ValueError('asset/grasp source changed')
 P=read(WORKSPACE/'configs/protocols/protocol_v1.json');source=case_dir(case_id)/STAGES[0];model=mujoco.MjModel.from_binary_path(str(source/'compiled_model.mjb'));reference=mujoco.MjData(model);mujoco.mj_setState(model,reference,np.load(source/'initial_snapshot.npz')['state'],mujoco.mjtState.mjSTATE_INTEGRATION)
 # 每次初始化除了三个底盘坐标，所有 qpos 都必须保持源快照。
 allowed={int(model.jnt_qposadr[model.joint('robot_0/'+j).id]) for j in ['base_x','base_y','base_theta']};mask=np.ones(model.nq,bool);mask[list(allowed)]=False;checks=[]
 for trial in read(folder/'physics.json'):
  run=(folder/trial['result_path']).parent;data=mujoco.MjData(model);mujoco.mj_setState(model,data,np.load(run/'placed_snapshot.npz')['state'],mujoco.mjtState.mjSTATE_INTEGRATION)
  unchanged=bool(np.allclose(reference.qpos[mask],data.qpos[mask],rtol=0,atol=1e-12) and np.allclose(reference.qvel,data.qvel,rtol=0,atol=1e-12))
  if not unchanged:raise ValueError('trial initial scene/robot nonbase state was modified: '+trial['trial_id'])
  recomputed=None
  if trial.get('trace_path'):
   path=folder/trial['trace_path']
   if digest(path)!=trial['trace_sha256']:raise ValueError('raw Pick trace hash mismatch')
   t=read(path);prefix=read(run/'torso_adjust_trace.json');pf=validate_motion_prefix(prefix['samples'],prefix['initial'],P['thresholds']);verdict=validate_trace(t['samples'],t['initial'],P['thresholds'])
   if pf:verdict={'status':'executed_failure','reason':pf}
   if verdict['status']=='infrastructure_error':raise ValueError('trace validation infrastructure error')
   recomputed=int(verdict['status']=='verified_success')
   if recomputed!=trial['binary_pick_label']:raise ValueError('label differs from original trace validation')
  elif trial['binary_pick_label'] is not None:raise ValueError('binary label without complete trace')
  checks.append({'trial_id':trial['trial_id'],'source_snapshot_restored_nonbase_qpos_and_all_qvel_unchanged':unchanged,'label_recomputed':recomputed})
 plans=read(folder/'planning.json');keys=[(x['pose_id'],x['side'],x['h']) for x in plans]
 if len(keys)!=len(set(keys)):raise ValueError('duplicate planning key')
 rows=read(folder/'candidates.json');config=read(folder/'frozen_inputs.json')['config'];required=sum(r['geometry_status']=='eligible' for r in rows)*len(config['planning']['arms'])*len(config['planning']['heights'])
 if len(plans)!=required:raise ValueError('incomplete multiheight/both-arm planning matrix')
 result={'case_id':case_id,'run_id':run_id,'all_checks_passed':True,'planning_unique_queries':len(plans),'required_planning_queries':required,'independent_trial_count':len(checks),'trial_checks':checks,'created_at_utc':now()};write(folder/'evidence_audit.json',result)
 names=['frozen_inputs.json','candidates.json','reach_bound.json','planning.json','physics_selection.json','physics.json','stations.json','station_table.csv','summary.json','evidence_audit.json','floor_sparse_samples.csv','floor_sparse_metadata.json','station_map.py']
 paths=[folder/name for name in names]+list((folder/'trials').glob('*/*.json'))+list((folder/'trials').glob('*/*.npz'))
 write(folder/'evidence_manifest.json',{'case_id':case_id,'run_id':run_id,'files':{str(p.relative_to(folder)):digest(p) for p in paths},'created_at_utc':now()})
 return result

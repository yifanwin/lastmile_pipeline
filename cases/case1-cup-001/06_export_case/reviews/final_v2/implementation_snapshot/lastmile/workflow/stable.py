"""步骤 2：复核原始稳定性证据，并在当前恢复的 A 点重新执行。"""
from .core import *
from ..admission import admit_asset
from ..validation import validate_motion_prefix,validate_trace

def revalidate():
 config=load_config('02_stable_grasp.yaml');P=read(WORKSPACE/'configs/protocols/protocol_v1.json');selection=WORKSPACE/config['import_evidence']['selection_protocol'];frozen=read(selection);trials=[];evidence=[]
 for kind in ['nominal','perturbed']:
  report=WORKSPACE/config['import_evidence'][kind]
  for j,row in enumerate(read(report)['trials']):
   if row['selection_protocol_sha256']!=digest(selection):raise ValueError('冻结 grasp 选择协议不一致')
   path=remap_legacy(row['trace_path']);trace=read(path)
   if kind=='nominal':prefix=trace['prefix_samples'];initial=trace['prefix_initial']
   else:
    if row['base_delta']!=frozen['trials'][j]['base_delta']:raise ValueError('扰动与预注册不符')
    p=read(path.parent/'torso_adjust_trace.json');prefix=p['samples'];initial=p['initial']
   if validate_motion_prefix(prefix,initial,P['thresholds']):raise ValueError('历史躯干调整未通过')
   trials.append({**row,'initial':trace['initial'],'trace':trace['samples']})
   evidence.append({'trial_id':row['trial_id'],'trace_path':str(path.relative_to(WORKSPACE)),'trace_sha256':digest(path),'kind':kind})
 verdict=admit_asset('Cup_30','ordinary',trials,digest(WORKSPACE/'configs/protocols/protocol_v1.json'),P)
 return verdict,evidence

def calibrate(case_id,provisional=False):
 review_packet_valid(case_id)
 if not provisional:require_review(case_id)
 folder=case_dir(case_id)/STAGES[1];folder.mkdir(parents=True,exist_ok=True)
 if (folder/'summary.json').exists():raise ValueError('已有步骤 2 结果；禁止覆盖，请使用新的 case_id')
 facts=read(case_dir(case_id)/STAGES[0]/'scene_facts.json')
 if facts['source_index']!=14 or facts['asset_id']!='Cup_30':raise ValueError('当前稳定性证据仅适用于 episode 14 / Cup_30')
 update(case_id,STAGES[1],status='running',physics_verified=False)
 verdict,evidence=revalidate();write(folder/'historical_revalidation.json',{'verdict':verdict,'traces':evidence,'revalidated_at_utc':now()})
 if verdict['status']!='admitted':raise ValueError('历史稳定性物理轨迹复核未通过')
 from .restore import physical_signature,render_card
 from ..simulation import Simulation
 from ..calibration_runner import run_one
 config=load_config('02_stable_grasp.yaml');context=case_dir(case_id)/'inputs/engine_context';run=folder/'fresh_A_witness';sim=Simulation.from_cached(context,14,case_dir(case_id)/STAGES[0],run)
 if physical_signature(sim.model)!=facts['physical_model_signature_sha256']:raise ValueError('当前 A 点物理模型与恢复卡不同')
 result=run_one(context,14,config['side'],config['h'],0,run,1,sim=sim,world_geometry=config['world_geometry'],time_dilation=config['time_dilation'],approach_overshoot=config['approach_overshoot_m'],grasp_ids=config['grasp_ids'])
 trace_path=run/'grasp_0794_trace.json';passed=False
 if trace_path.exists():
  t=read(trace_path);p=read(run/'torso_adjust_trace.json');P=read(context/'configs/protocol_v1.json')
  passed=not validate_motion_prefix(p['samples'],p['initial'],P['thresholds']) and validate_trace(t['samples'],t['initial'],P['thresholds'])['status']=='verified_success'
 try:require_review(case_id);approved=True
 except ValueError:approved=False
 summary={'status':('verified' if approved else 'provisional_verified') if passed else 'physics_failed','physics_verified':passed,'point_name':'A','point_semantics':'initial_success_station_not_failure_start','base':facts['initial_base'],'asset_id':'Cup_30','source_index':14,'side':'left','h':.738,'grasp_ids':[794],'historical_stability':verdict,'fresh_A_witness':result,'fresh_successes':int(passed),'fresh_trials':1,'historical_counts_are_not_new_runs':True,'human_approved':approved,'scope':'A 点抓取；未验证其他站位、导航或 Case 1 完整闭环','protocol_sha256':digest(context/'configs/protocol_v1.json'),'completed_at_utc':now()}
 if trace_path.exists():summary['fresh_trace_sha256']=digest(trace_path)
 write(folder/'summary.json',summary);write(folder/'stable_grasps.json',{k:summary[k] for k in ['asset_id','side','h','grasp_ids','base','physics_verified','protocol_sha256']});seal_evidence(case_id);update(case_id,STAGES[1],status=summary['status'],physics_verified=passed);render_card(case_id);return summary


def seal_evidence(case_id):
 folder=case_dir(case_id)/STAGES[1]
 names=['historical_revalidation.json','stable_grasps.json','fresh_A_witness/result.json','fresh_A_witness/grasp_0794_trace.json','fresh_A_witness/torso_adjust_trace.json','fresh_A_witness/initial_snapshot.npz','fresh_A_witness/compiled_model.mjb']
 write(folder/'physics_evidence.json',{'schema_version':1,'case_id':case_id,'files':{name:digest(folder/name) for name in names if (folder/name).exists()},'sealed_at_utc':now()})

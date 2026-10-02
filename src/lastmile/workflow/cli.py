import argparse,json
from .core import *
def main():
 p=argparse.ArgumentParser(description='半自动 Lastmile：自动物理证据 + 人工门禁');s=p.add_subparsers(dest='command',required=True)
 for name in ['restore','agent-review','calibrate-a','approve','serve-review','status','check-gate','station-map','plot-station-map','audit-station-map','serve-station-map','construct-original','check-construct-gate','construct-robot-start','closed-loop','render-closed-loop','audit-closed-loop','prepare-final','accept-final','check-final-gate','export-case','verify-export','serve-final-review','vla-station-test']:
  q=s.add_parser(name);q.add_argument('--case-id',default='case1-cup-001')
  if name in ['station-map','plot-station-map','audit-station-map','serve-station-map','construct-original','check-construct-gate']:q.add_argument('--run-id',default='coarse_v1')
  if name in ['construct-robot-start','closed-loop','render-closed-loop','audit-closed-loop']:q.add_argument('--run-id',default='robot_move_v1')
  if name in ['closed-loop','render-closed-loop']:q.add_argument('--construction-run-id',default='robot_move_v1')
  if name=='closed-loop':
   q.add_argument('--trials',type=int,default=3);q.add_argument('--seed-offset',type=int,default=0)
  if name in ['prepare-final','accept-final','check-final-gate','export-case','serve-final-review']:q.add_argument('--review-id',default='final_v1')
  if name=='prepare-final':q.add_argument('--closed-loop-run-id',default='astar_v2')
  if name=='accept-final':q.add_argument('--file',required=True)
  if name=='export-case':
   q.add_argument('--export-id',default='final_v1');q.add_argument('--draft',action='store_true')
  if name=='verify-export':q.add_argument('--bundle',required=True)
  if name=='serve-final-review':q.add_argument('--port',type=int,default=18767)
  if name=='station-map':q.add_argument('--resume',action='store_true')
  if name=='vla-station-test':
   q.add_argument('--station-run-id',default='coarse_v1');q.add_argument('--run-id',default='molmobot_multitask_xy_v1')
   q.add_argument('--checkpoint-path');q.add_argument('--resume',action='store_true')
  if name=='construct-original':q.add_argument('--choice-file')
  if name=='restore':q.add_argument('--source-index',type=int,default=14)
  if name=='calibrate-a':q.add_argument('--provisional',action='store_true')
  if name=='approve':q.add_argument('--file',required=True)
  if name=='serve-review':q.add_argument('--port',type=int,default=8765)
  if name=='serve-station-map':q.add_argument('--port',type=int,default=18766)
 a=p.parse_args()
 try:
  if a.command=='vla-station-test':
   from .vla_station import run_vla_station_test
   result=run_vla_station_test(a.case_id,a.station_run_id,a.run_id,a.checkpoint_path,a.resume)
  elif a.command=='restore':
   from .restore import run_restore
   result=run_restore(a.case_id,a.source_index)
  elif a.command=='agent-review':
   review_packet_valid(a.case_id)
   from .agent import review_scene
   from .restore import render_card
   result=review_scene(a.case_id);render_card(a.case_id)
  elif a.command=='calibrate-a':
   from .stable import calibrate
   result=calibrate(a.case_id,a.provisional)
  elif a.command=='approve':
   decision=read(a.file)
   if decision.get('case_id')!=a.case_id:raise ValueError('人工确认 case_id 不匹配')
   result=confirm_review(a.case_id,decision)
   from .restore import render_card
   render_card(a.case_id)
  elif a.command=='serve-review':
   from .server import serve
   serve(a.case_id,a.port);return
  elif a.command=='station-map':
   from .station_map import run_station_map
   result=run_station_map(a.case_id,a.run_id,a.resume)
  elif a.command=='serve-station-map':
   from .server import serve_map
   serve_map(a.case_id,a.run_id,a.port);return
  elif a.command=='audit-station-map':
   from .station_audit import audit_station_map
   result=audit_station_map(a.case_id,a.run_id)
  elif a.command=='plot-station-map':
   from .station_plot import export_map
   result=export_map(a.case_id,a.run_id)
  elif a.command=='construct-original':
   from .construction import run_original_screen
   result=run_original_screen(a.case_id,a.choice_file)
  elif a.command=='construct-robot-start':
   from .hard_start import construct_robot_start
   result=construct_robot_start(a.case_id,a.run_id)
  elif a.command=='closed-loop':
   from .closed_loop import run_closed_loop
   result=run_closed_loop(a.case_id,a.construction_run_id,a.run_id,a.trials,seed_offset=a.seed_offset) if a.seed_offset else run_closed_loop(a.case_id,a.construction_run_id,a.run_id,a.trials)
  elif a.command=='audit-closed-loop':
   from .closed_loop_audit import audit_closed_loop
   result=audit_closed_loop(a.case_id,a.run_id)
  elif a.command=='render-closed-loop':
   from .closed_loop_video import render_closed_loop
   result=render_closed_loop(a.case_id,a.run_id)
  elif a.command=='prepare-final':
   from .final_review import prepare_final_review
   result=prepare_final_review(a.case_id,a.closed_loop_run_id,a.review_id)
  elif a.command=='accept-final':
   from .final_review import confirm_final_review
   result=confirm_final_review(a.case_id,a.review_id,read(a.file))
  elif a.command=='check-final-gate':
   from .final_review import require_final_acceptance
   require_final_acceptance(a.case_id,a.review_id);result={'can_export_accepted_case':True}
  elif a.command=='export-case':
   from .case_export import assemble_case
   result=assemble_case(a.case_id,a.review_id,a.export_id,a.draft)
  elif a.command=='verify-export':
   from .case_export import verify_export_bundle
   result=verify_export_bundle(a.bundle)
  elif a.command=='serve-final-review':
   from .server import serve_final_review
   serve_final_review(a.case_id,a.review_id,a.port);return
  elif a.command=='check-construct-gate':
   from .construction import require_constructed_candidate
   result={'can_enter_step5':require_constructed_candidate(a.case_id)}
  elif a.command=='check-gate':result={'can_enter_step3':require_next_stage(a.case_id),'step3_implemented':True}
  else:result=manifest(a.case_id)
  print(json.dumps(result,ensure_ascii=False,indent=2))
 except Exception as exc:
  if a.command=='calibrate-a':
   m=manifest(a.case_id)
   if m['stages'][STAGES[1]].get('status')=='running':update(a.case_id,STAGES[1],status='infrastructure_error',physics_verified=False,error_type=type(exc).__name__)
  if a.command=='station-map':
   update(a.case_id,STAGES[2],status='infrastructure_error',error_type=type(exc).__name__,map_ready=False)
  if a.command=='construct-original' and manifest(a.case_id)['stages'][STAGES[3]].get('status')=='screening':
   update(a.case_id,STAGES[3],status='infrastructure_error',can_enter_step5=False,error_type=type(exc).__name__)
  if a.command=='closed-loop' and manifest(a.case_id)['stages'][STAGES[4]].get('status')=='running':
   update(a.case_id,STAGES[4],status='infrastructure_error',closed_loop_verified=False,error_type=type(exc).__name__)
  # 不输出 HTTP 详情和凭据；API 错误由 agent.py 负责脱敏。
  if a.command=='agent-review':message=type(exc).__name__+'：agent 请求未完成，请检查本地配置'
  else:message=str(exc)
  p.exit(1,message+'\n')
if __name__=='__main__':main()

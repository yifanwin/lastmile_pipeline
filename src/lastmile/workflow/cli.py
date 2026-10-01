import argparse,json
from .core import *
def main():
 p=argparse.ArgumentParser(description='半自动 Lastmile：自动物理证据 + 人工门禁');s=p.add_subparsers(dest='command',required=True)
 for name in ['restore','agent-review','calibrate-a','approve','serve-review','status','check-gate']:
  q=s.add_parser(name);q.add_argument('--case-id',default='case1-cup-001')
  if name=='restore':q.add_argument('--source-index',type=int,default=14)
  if name=='calibrate-a':q.add_argument('--provisional',action='store_true')
  if name=='approve':q.add_argument('--file',required=True)
  if name=='serve-review':q.add_argument('--port',type=int,default=8765)
 a=p.parse_args()
 try:
  if a.command=='restore':
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
  elif a.command=='check-gate':result={'can_enter_step3':require_next_stage(a.case_id),'step3_implemented':False}
  else:result=manifest(a.case_id)
  print(json.dumps(result,ensure_ascii=False,indent=2))
 except Exception as exc:
  if a.command=='calibrate-a':
   m=manifest(a.case_id)
   if m['stages'][STAGES[1]].get('status')=='running':update(a.case_id,STAGES[1],status='infrastructure_error',physics_verified=False,error_type=type(exc).__name__)
  # 不输出 HTTP 详情和凭据；API 错误由 agent.py 负责脱敏。
  if a.command=='agent-review':message=type(exc).__name__+'：agent 请求未完成，请检查本地配置'
  else:message=str(exc)
  p.exit(1,message+'\n')
if __name__=='__main__':main()

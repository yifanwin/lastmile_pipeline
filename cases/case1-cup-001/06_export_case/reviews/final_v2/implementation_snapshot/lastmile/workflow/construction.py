"""步骤 4：记录人工构造方式，并按证据筛选；不把成功起点改标失败。"""
from pathlib import Path
import json,html,base64
import numpy as np
from . import core as c

def same_pose(base,reference,xy_tolerance=1e-5,yaw_tolerance=1e-5):
 a=np.asarray(base,float);b=np.asarray(reference,float)
 if a.shape!=(3,) or b.shape!=(3,) or not np.isfinite(a).all() or not np.isfinite(b).all():return False
 angle=np.arctan2(np.sin(a[2]-b[2]),np.cos(a[2]-b[2]))
 return bool(np.linalg.norm(a[:2]-b[:2])<=xy_tolerance and abs(angle)<=yaw_tolerance)

def evaluate_original_start(base,verified_witnesses):
 matching=[w for w in verified_witnesses if same_pose(w['base'],base)]
 success=[w for w in matching if w['verdict']['status']=='verified_success']
 if success:return {'status':'screened_out_original_start_success','screening_complete':True,'candidate_selected':False,'can_enter_step5':False,'reason':'original_start_has_protocol_compliant_pick_witness','witness_ids':[w['witness_id'] for w in success]}
 # 没有成功证据不等于失败，也不能把单次执行失败升级为全 yaw / 双臂 / 连续 h 不可达。
 return {'status':'needs_original_start_failure_evidence','screening_complete':False,'candidate_selected':False,'can_enter_step5':False,'reason':'absence_of_success_is_not_failure_or_distance_unreachability','witness_ids':[]}

def validate_human_method(choice,case_id):
 if choice.get('case_id')!=case_id:raise ValueError('人工构造选择 case_id 不匹配')
 if choice.get('actor')!='human' or choice.get('decision')!='method_selected':raise ValueError('构造方式必须由人工选择，agent 不能批准')
 if choice.get('method')!='original_only':raise ValueError('当前选择为仅筛选原始起点；其他构造分支尚未实现，禁止自动改方式')
 if choice.get('allow_scene_edits')!=[]:raise ValueError('原样筛选不允许任何场景或机器人初始位姿修改')
 return choice

def checked_file(root,name,sha):
 root=Path(root).resolve();p=(root/name).resolve()
 if not p.is_relative_to(root):raise ValueError('证据路径越界')
 if c.digest(p)!=sha:raise ValueError('站位图证据已改变：'+name)
 return p

def require_station_evidence(case_id,run_id):
 c.case_dir(run_id);c.require_next_stage(case_id)
 m=c.manifest(case_id)
 if m['stages'][c.STAGES[2]].get('status')!='coarse_map_complete' or not m['stages'][c.STAGES[2]].get('map_ready'):raise ValueError('步骤 3 尚未完成')
 root=c.case_dir(case_id)/c.STAGES[2]/'runs'/run_id;packet=c.read(root/'evidence_manifest.json')
 if packet.get('case_id')!=case_id or packet.get('run_id')!=run_id:raise ValueError('站位图来源不匹配')
 for name,sha in packet['files'].items():checked_file(root,name,sha)
 audit=c.read(root/'evidence_audit.json')
 if not audit.get('all_checks_passed'):raise ValueError('站位图原始证据审计未通过')
 return root

def run_original_screen(case_id,choice_file=None):
 folder=c.case_dir(case_id)/c.STAGES[3];folder.mkdir(parents=True,exist_ok=True)
 choice=c.read(choice_file or folder/'human_choice.json');validate_human_method(choice,case_id);config=c.load_config('04_construct_start.yaml')
 if config['method']!='original_only' or config['allowed_scene_edits'] or any(config[k] for k in ['allow_robot_initial_pose_change','allow_target_pose_change','allow_furniture_change','allow_asset_replacement']):raise ValueError('配置违反人工选定的原样筛选约束')
 if (folder/'summary.json').exists():raise ValueError('步骤 4 已有筛选结果，禁止覆盖；用 check-construct-gate 查看状态')
 station=require_station_evidence(case_id,config['source_station_run']);episode=c.read(c.case_dir(case_id)/'inputs/source_episode.json');base=episode['robot']['init_qpos']['base'];P=c.read(c.WORKSPACE/'configs/protocols/protocol_v1.json')
 from ..validation import validate_trace,validate_motion_prefix
 witnesses=[];c.update(case_id,c.STAGES[3],status='screening',method='original_only',human_method_selected=True)
 for record in c.read(station/'physics.json'):
  if not same_pose(record['base'],base,config['original_pose_match_translation_m'],config['original_pose_match_yaw_rad']) or not record.get('trace_path'):continue
  trace_path=station/record['trace_path'];trace=c.read(trace_path);prefix_path=trace_path.parent/'torso_adjust_trace.json';prefix=c.read(prefix_path)
  if trace['initial']['target_id']!=episode['task']['pickup_obj_name'] or not same_pose(trace['initial']['base'],base):raise ValueError('原始起点 witness 的目标/底盘错配')
  if c.digest(trace_path)!=record['trace_sha256']:raise ValueError('原始起点轨迹哈希错配')
  failure=validate_motion_prefix(prefix['samples'],prefix['initial'],P['thresholds']);verdict=validate_trace(trace['samples'],trace['initial'],P['thresholds'])
  if failure:verdict={'status':'executed_failure','reason':failure}
  if verdict['status']=='infrastructure_error':raise ValueError('原始起点轨迹验收基础设施异常')
  witnesses.append({'witness_id':record['trial_id'],'base':record['base'],'side':record['side'],'h':record['h'],'grasp_id':record['grasp_id'],'verdict':verdict,'trace_path':str(trace_path.relative_to(c.WORKSPACE)),'trace_sha256':c.digest(trace_path),'prefix_path':str(prefix_path.relative_to(c.WORKSPACE)),'prefix_sha256':c.digest(prefix_path),'new_execution':False})
 result=evaluate_original_start(base,witnesses)
 saved={**choice,'review_packet_sha256':c.review_packet_valid(case_id),'source_episode_sha256':c.digest(c.case_dir(case_id)/'inputs/source_episode.json'),'station_evidence_manifest_sha256':c.digest(station/'evidence_manifest.json'),'captured_at_utc':c.now(),'candidate_approval':False}
 c.write(folder/'human_choice.json',saved)
 summary={**result,'case_id':case_id,'source_index':episode['provenance']['source_index'],'method':'original_only','original_base':base,'scene_edits':[],'robot_initial_pose_changed':False,'target_pose_changed':False,'furniture_changed':False,'new_physical_trials':0,'physics_witnesses_revalidated':witnesses,'source_episode_sha256':saved['source_episode_sha256'],'review_packet_sha256':saved['review_packet_sha256'],'station_run_id':config['source_station_run'],'station_evidence_manifest_sha256':saved['station_evidence_manifest_sha256'],'human_method_choice_sha256':c.digest(folder/'human_choice.json'),'policy_config_sha256':c.digest(c.WORKSPACE/'configs/04_construct_start.yaml'),'case1_goal_achieved':False,'closed_loop_verified':False,'completed_at_utc':c.now()}
 c.write(folder/'summary.json',summary)
 # 不创建失败起点 config 或 step5 execute 请求；只保留原状态来源和筛除清单。
 c.write(folder/'screening_registry.json',{'case_id':case_id,'method':'original_only','accepted_candidates':[],'excluded_candidates':[{'source_index':summary['source_index'],'reason':result['reason'],'evidence':result['witness_ids']}] if result['screening_complete'] else [],'pending_candidates':[] if result['screening_complete'] else [case_id]})
 c.update(case_id,c.STAGES[3],status=result['status'],method='original_only',human_method_selected=True,screening_complete=result['screening_complete'],candidate_selected=False,can_enter_step5=False,scene_edits=[])
 seal_decision(case_id)
 render_construction(case_id)
 return summary

def seal_decision(case_id):
 folder=c.case_dir(case_id)/c.STAGES[3]
 c.write(folder/'decision_packet.json',{'case_id':case_id,'files':{name:c.digest(folder/name) for name in ['summary.json','human_choice.json','screening_registry.json']}})

def require_constructed_candidate(case_id):
 latest=c.case_dir(case_id)/c.STAGES[3]/'latest.json'
 if latest.exists() and c.read(latest).get('method')=='robot_only':
  from .hard_start import require_robot_candidate
  require_robot_candidate(case_id,c.read(latest)['run_id']);return True
 folder=c.case_dir(case_id)/c.STAGES[3];packet=c.read(folder/'decision_packet.json')
 if packet.get('case_id')!=case_id:raise ValueError('筛选包 case_id 错配')
 for name,sha in packet['files'].items():checked_file(folder,name,sha)
 summary=c.read(folder/'summary.json');choice=c.read(folder/'human_choice.json');validate_human_method(choice,case_id)
 if c.digest(folder/'human_choice.json')!=summary['human_method_choice_sha256']:raise ValueError('人工构造选择已改变，筛选失效')
 if c.digest(c.WORKSPACE/'configs/04_construct_start.yaml')!=summary['policy_config_sha256']:raise ValueError('构造政策已改变，筛选失效')
 c.require_next_stage(case_id)
 if c.digest(c.case_dir(case_id)/'inputs/source_episode.json')!=summary['source_episode_sha256']:raise ValueError('原始 episode 已改变')
 if c.review_packet_valid(case_id)!=summary['review_packet_sha256']:raise ValueError('恢复证据已改变')
 root=require_station_evidence(case_id,summary['station_run_id'])
 if c.digest(root/'evidence_manifest.json')!=summary['station_evidence_manifest_sha256']:raise ValueError('站位图证据版本已改变')
 for witness in summary['physics_witnesses_revalidated']:
  if c.digest(c.WORKSPACE/witness['trace_path'])!=witness['trace_sha256'] or c.digest(c.WORKSPACE/witness['prefix_path'])!=witness['prefix_sha256']:raise ValueError('原始起点物理证据已改变')
 if not summary['candidate_selected'] or not summary['can_enter_step5']:
  raise ValueError('步骤 5 未放行：原始起点已成功抓取，不能作为 Case 1 失败起点' if summary['status']=='screened_out_original_start_success' else '步骤 5 未放行：原始起点失败证据不足')
 # 初版不接受手改 summary 来覆盖物理通过；后续分支需独立实现审核。
 raise ValueError('当前原样筛选分支无已验收的困难候选，禁止手工覆盖为放行')

def render_construction(case_id):
 folder=c.case_dir(case_id)/c.STAGES[3];summary=c.read(folder/'summary.json');choice=c.read(folder/'human_choice.json');esc=html.escape
 excluded=summary['status']=='screened_out_original_start_success'
 headline=f'筛除当前 episode {summary["source_index"]}：原始起点已真实抓取成功，不能作为 Case 1 的失败起点。' if excluded else '原始起点失败证据不足：尚未选中困难候选。'
 evidence_note='原始起点的严格抓取成功足以反驳其不可操作；不会用其他站位的失败冒充原始起点失败。' if excluded else '没有成功证据不等于失败，规划搜索耗尽也不能证明不可达。'
 source=c.case_dir(case_id)/c.STAGES[0]/'views/initial_robot_view.png';img=base64.b64encode(source.read_bytes()).decode();links=''
 for w in summary['physics_witnesses_revalidated']:
  links+=f'<li>{esc(w["witness_id"])}：{esc(w["side"])} / h={w["h"]} / grasp {w["grasp_id"]} → {esc(w["verdict"]["status"])}；<code>{esc(w["trace_path"])}</code></li>'
 text=f'''<!doctype html><html lang="zh"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>步骤 4 原样筛选</title><style>body{{font:17px/1.7 system-ui;max-width:1100px;margin:28px auto;padding:20px;background:#f5f7f9;color:#223}}section{{background:white;border-radius:12px;padding:24px;margin:20px 0}}.warning{{border-left:5px solid #df9a28}}img{{width:100%;height:auto}}pre,code{{white-space:pre-wrap;overflow-wrap:anywhere}}a{{color:#2673a5}}</style><h1>{esc(case_id)} · 步骤 4：仅筛选原始起点</h1><section class="warning"><b>{esc(headline)}</b><p>已按你的人工选择保存 original_only；未移动机器人、目标或家具，未替换资产。筛选完成不等于 Case 1 已构造成功；步骤 5 没有放行。</p></section><section><h2>人工选择</h2><p>{esc(choice.get('user_answer','仅筛选原始起点，不改场景'))}</p><p>原始 base：<code>{esc(str(summary['original_base']))}</code>。允许的躯干调整和机械臂动作属于执行协议，不是修改原始场景。</p><img src="data:image/png;base64,{img}" alt="原始机器人初始视角"><p>沿用步骤 1 的原始视角，不是新构造起点。</p></section><section><h2>重新验收的原始起点物理证据</h2><ul>{links}</ul><p>{esc(evidence_note)}</p></section><section><h2>下一步</h2><p>保持原样筛选政策时，应继续筛选其他原始 episode；新场景仍需来源审计、场景确认、稳定抓法和站位图。当前成功例作为正对照保留，不丢弃物理证据。若之后改变构造方式，需要你重新明确选择，不能偷偷移动机器人。</p><p><a href="summary.json">筛选结果 JSON</a> · <a href="human_choice.json">人工方式选择记录</a> · <a href="screening_registry.json">筛除清单</a></p></section></html>'''
 (folder/'construction_review.html').write_text(text)
 (folder/'README.md').write_text(f'# 步骤 4：原样筛选\n\n{headline} 步骤 5 未放行；机器人起点、目标和场景不改。\n\n[审阅卡](construction_review.html) · [筛选结果](summary.json)\n')

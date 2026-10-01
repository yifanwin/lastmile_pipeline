"""最终人工审阅和不可变证据门禁；接受构造不等于改写物理标签。"""
from pathlib import Path
import base64,html,json
from . import core as c
from .construction import checked_file
from .hard_start import require_robot_candidate

CHECKS=['watched_closed_loop_video','accept_start_failure_evidence','accept_robot_initial_base_edit','accept_translation_only_classification','accept_three_trial_requirement']

def validate_final_decision(decision,packet,packet_sha):
    if decision.get('case_id')!=packet['case_id'] or decision.get('review_id')!=packet['review_id'] or decision.get('review_packet_sha256')!=packet_sha:raise ValueError('最终人工决定与审阅证据包不匹配/过期')
    if decision.get('actor')!='human':raise ValueError('最终接受必须来自人工，agent 无批准权')
    if decision.get('decision') not in ['accepted','rejected','reconstruct']:raise ValueError('请选择 accepted / rejected / reconstruct；pending 不放行')
    if not isinstance(decision.get('reviewer'),str) or not decision['reviewer'].strip():raise ValueError('请填写人工审阅者')
    if decision['decision']=='accepted':
        if any(decision.get(k) is not True for k in CHECKS):raise ValueError('接受前必须明确确认视频、起点证据、修改、同侧平移分类及三次要求')
        if not packet['qualification']['physics_qualified']:raise ValueError('人工不能覆盖未通过的物理验收')
    return decision

def verify_file_map(root,files):
    for name,sha in files.items():checked_file(root,name,sha)

def validate_repeat_summary(summary,required):
    trials=summary.get('trials',[])
    if not isinstance(required,int) or isinstance(required,bool) or required<1:raise ValueError('invalid repeat requirement')
    if len(trials)!=summary.get('trials_completed') or len(trials)!=summary.get('trials_requested') or len(trials)<required:raise ValueError('完整链路试验数不足/不完整')
    ids=[r.get('trial') for r in trials]
    if None in ids or len(set(ids))!=len(ids):raise ValueError('重复/缺失试验标识')
    if any(r.get('status')!='verified_success' or r.get('new_physical_execution') is not True or r.get('arrival_used_without_reset') is not True for r in trials):raise ValueError('完整链路并非全部新执行且严格通过，禁止人工覆盖')
    if summary.get('strict_successes')!=len(trials) or summary.get('closed_loop_verified') is not True:raise ValueError('闭环汇总与试验不一致')
    return {'physics_qualified':True,'full_chain_trials':len(trials),'full_chain_strict_successes':len(trials),'required_trials':required,'all_trials_successful':True}

def verified_inputs(case_id,run_id,reaudit=False):
    c.case_dir(run_id);case=c.case_dir(case_id);root=case/c.STAGES[4]/'runs'/run_id
    c.require_next_stage(case_id)
    if reaudit:
        from .closed_loop_audit import audit_closed_loop
        audit_closed_loop(case_id,run_id)
    physics=c.read(root/'evidence_manifest.json')
    if physics['case_id']!=case_id or physics['run_id']!=run_id:raise ValueError('闭环证据来源错配')
    verify_file_map(root,physics['files']);frozen=c.read(root/'frozen_inputs.json');construction,config=require_robot_candidate(case_id,frozen['construction_run_id'])
    if c.digest(construction/'evidence_manifest.json')!=frozen['construction_manifest_sha256']:raise ValueError('构造证据版本已改变')
    policy=c.load_config('06_export_case.yaml')
    if policy['human_final_acceptance_required'] is not True or policy['allow_human_override_physical_failure'] is not False:raise ValueError('导出政策禁止取消人工确认或覆盖物理失败')
    need=policy['full_chain'];qualification=validate_repeat_summary(c.read(root/'summary.json'),need['required_trials'])
    if need['required_successes']!=need['required_trials']:raise ValueError('当前首版要求全部成功')
    if need['required_trials']<need['default_plan_min_trials']:
        override=need['policy_override']
        if override.get('actor')!='human' or override.get('source')!='explicit_user_chat_request' or override.get('scope')!='this_case_full_chain_repeats_only':raise ValueError('降低重复门槛必须有明确人工要求')
    audit=c.read(root/'evidence_audit.json')
    if not audit['all_checks_passed'] or audit['strict_successes_recomputed']!=qualification['full_chain_trials'] or len(audit['trials'])!=qualification['full_chain_trials']:raise ValueError('原始闭环证据复核不完整')
    stable=c.read(case/c.STAGES[1]/'summary.json');counts=stable['historical_stability'];protocol=c.read(c.WORKSPACE/'configs/protocols/protocol_v1.json');ad=protocol['admission']
    if counts['nominal_trials']<ad['nominal_trials'] or counts['nominal_successes']<ad['nominal_successes'] or counts['perturbed_trials']<ad['perturbed_trials'] or counts['perturbed_successes']<ad['perturbed_successes']:raise ValueError('资产名义/扰动稳定性证据不足')
    delivery=root/'delivery';dpacket=c.read(delivery/'delivery_manifest.json');verify_file_map(delivery,dpacket['files'])
    vm=c.read(delivery/'video_manifest.json')
    if dpacket['physics_evidence_manifest_sha256']!=c.digest(root/'evidence_manifest.json') or vm['physical_evidence_manifest_sha256']!=c.digest(root/'evidence_manifest.json'):raise ValueError('视频未绑定当前物理证据')
    if c.digest(delivery/'case1_r_closed_loop.mp4')!=vm['video_sha256']:raise ValueError('视频哈希错配')
    qualification.update(nominal_asset_evidence=counts,historical_asset_counts_are_not_new_runs=True,default_plan_full_chain_trials=need['default_plan_min_trials'],policy_override=need['policy_override'])
    return root,construction,config,policy,qualification

def gather_sources(case_id,root,construction):
    case=c.case_dir(case_id);files={}
    def add(path,sha=None):
        path=Path(path);relative=str(path.absolute().relative_to(c.WORKSPACE));checked_file(c.WORKSPACE,relative,sha or c.digest(path));files[relative]=sha or c.digest(path)
    for name in ['source_episode.json','asset.json']:add(case/'inputs'/name)
    first=case/c.STAGES[0];p=c.read(first/'review_packet.json')
    for name,sha in p['files'].items():add(first/name,sha)
    for name,sha in [('compiled_model.mjb',p['compiled_model_sha256']),('initial_snapshot.npz',p['initial_snapshot_sha256'])]:add(first/name,sha)
    for name in ['review_packet.json','human_review.json','restoration.json']:add(first/name)
    stable=case/c.STAGES[1];p=c.read(stable/'physics_evidence.json')
    for name,sha in p['files'].items():add(stable/name,sha)
    for name in ['physics_evidence.json','summary.json']:add(stable/name)
    for row in c.read(stable/'historical_revalidation.json')['traces']:add(c.WORKSPACE/row['trace_path'],row['trace_sha256'])
    station=case/c.STAGES[2]/'runs'/'coarse_v1'
    for name in ['station_map.png','station_map.svg','station_map.html']:add(station/name)
    for folder,manifest_name in [(station,'evidence_manifest.json'),(construction,'evidence_manifest.json'),(root,'evidence_manifest.json'),(root/'delivery','delivery_manifest.json')]:
        p=c.read(folder/manifest_name)
        for name,sha in p['files'].items():add(folder/name,sha)
        add(folder/manifest_name)
    for path in [c.WORKSPACE/'configs/protocols/protocol_v1.json',c.WORKSPACE/'configs/04_robot_start.yaml',c.WORKSPACE/'configs/06_export_case.yaml']:add(path)
    # 当前实现快照与当时执行代码哈希分别保留；不把当前代码冒充原始版本。
    return files

def review_folder(case_id,review_id):
    c.case_dir(review_id);return c.case_dir(case_id)/c.STAGES[5]/'reviews'/review_id

def require_review_packet(case_id,review_id):
    folder=review_folder(case_id,review_id);packet=c.read(folder/'review_packet.json')
    latest=c.case_dir(case_id)/c.STAGES[5]/'latest.json'
    if latest.exists() and c.read(latest)['review_id']!=review_id:raise ValueError('当前已有更新审阅包，旧审阅不再放行；历史证据保留')
    if packet['case_id']!=case_id or packet['review_id']!=review_id:raise ValueError('最终审阅包来源错配')
    verify_file_map(c.WORKSPACE,packet['source_files']);verify_file_map(folder,packet['review_files'])
    root,construction,config,policy,q=verified_inputs(case_id,packet['closed_loop_run_id'])
    if q!=packet['qualification']:raise ValueError('物理资格或重复要求已变化，旧人工决定失效')
    return folder,packet,c.digest(folder/'review_packet.json')

def prepare_final_review(case_id,run_id='astar_v2',review_id='final_v1'):
    folder=review_folder(case_id,review_id)
    if folder.exists():raise ValueError('审阅包不可覆盖；请用新 review-id')
    root,construction,config,policy,qualification=verified_inputs(case_id,run_id,reaudit=True)
    folder.mkdir(parents=True);files=gather_sources(case_id,root,construction)
    draft={'schema_version':1,'case_id':case_id,'status':'draft_pending_human_acceptance','dataset_admitted':False,'human_final_accepted':False,'case_type':'Case 1-R','classification':{'base_translation_required':True,'same_side_translation_case':True,'must_change_table_side':False,'same_side_all_stations_unreachable':False,'start_failure_strength':'conservative_contact_bound_both_arms_all_yaw_continuous_legal_torso','fixed_base_failure_action_executed':False},'source':c.read(c.case_dir(case_id)/'inputs/source_episode.json')['provenance'],'construction_run_id':construction.name,'closed_loop_run_id':run_id,'start_pose_id':config['start_pose_id'],'start_base':config['start_base'],'goal_pose_id':'A','goal_base':config['goal_base'],'staging_base':config['staging_base'],'scene_edits':config['scene_edits'],'qualification':qualification,'task_language':'移动到可操作站位，抓起目标杯子并稳定保持；不做放置。','contents_simulated':False,'not_a_population_success_rate_estimate':True,'scope':'three accepted full-chain trials; original model and grasp; no same-side-wide impossibility claim','created_at_utc':c.now()}
    c.write(folder/'case_draft.json',draft)
    (folder/'reference_plan.md').write_bytes((c.WORKSPACE.parent/'lastmile case 构建方案.md').read_bytes())
    packet={'schema_version':1,'case_id':case_id,'review_id':review_id,'closed_loop_run_id':run_id,'construction_run_id':construction.name,'qualification':qualification,'source_files':files,'review_files':{'case_draft.json':c.digest(folder/'case_draft.json'),'reference_plan.md':c.digest(folder/'reference_plan.md')},'policy_override':policy['full_chain']['policy_override'],'created_at_utc':c.now()}
    for source in (c.WORKSPACE/'src/lastmile').rglob('*.py'):
        dest=folder/'implementation_snapshot'/source.relative_to(c.WORKSPACE/'src');dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(source.read_bytes());packet['review_files'][str(dest.relative_to(folder))]=c.digest(dest)
    c.write(folder/'review_packet.json',packet);sha=c.digest(folder/'review_packet.json')
    pending={'case_id':case_id,'review_id':review_id,'review_packet_sha256':sha,'actor':'human','decision':'pending','reviewer':'',**{k:False for k in CHECKS},'notes':''}
    c.write(folder/'human_review.json',pending)
    c.write(c.case_dir(case_id)/c.STAGES[5]/'latest.json',{'review_id':review_id,'closed_loop_run_id':run_id,'review_packet_sha256':sha})
    c.update(case_id,c.STAGES[5],status='pending_human_acceptance',review_id=review_id,closed_loop_run_id=run_id,physics_qualified=True,required_full_chain_trials=qualification['required_trials'],human_final_accepted=False,exported=False)
    render_final_review(case_id,review_id)
    return {'review_id':review_id,'review_packet_sha256':sha,'physics_qualified':True,'full_chain_strict_successes':qualification['full_chain_strict_successes'],'human_final_accepted':False,'review_html':str((folder/'review.html').relative_to(c.WORKSPACE)),'human_review_json':str((folder/'human_review.json').relative_to(c.WORKSPACE))}

def confirm_final_review(case_id,review_id,decision):
    folder,packet,sha=require_review_packet(case_id,review_id);validate_final_decision(decision,packet,sha)
    previous=folder/'accepted_review.json'
    if previous.exists():raise ValueError('已有不可变人工决定；需新的审阅包，不能覆盖')
    saved={**decision,'actor':'human','submitted_at_utc':c.now()};c.write(previous,saved)
    c.update(case_id,c.STAGES[5],status='human_accepted_ready_to_export' if decision['decision']=='accepted' else 'reconstruction_requested' if decision['decision']=='reconstruct' else 'human_rejected',human_final_accepted=decision['decision']=='accepted',exported=False,human_decision_sha256=c.digest(previous))
    render_final_review(case_id,review_id);return saved

def require_final_acceptance(case_id,review_id):
    folder,packet,sha=require_review_packet(case_id,review_id)
    if not (folder/'accepted_review.json').exists():raise ValueError('最终人工确认尚未提交，正式导出未放行')
    decision=c.read(folder/'accepted_review.json');validate_final_decision(decision,packet,sha)
    if decision['decision']!='accepted':raise ValueError('最终人工未接受，不允许正式导出')
    stage=c.manifest(case_id)['stages'][c.STAGES[5]]
    if stage.get('review_id')!=review_id or stage.get('human_decision_sha256')!=c.digest(folder/'accepted_review.json'):raise ValueError('人工记录已改变或不属于当前审阅')
    return folder,packet,decision

def render_final_review(case_id,review_id):
    folder=review_folder(case_id,review_id);packet=c.read(folder/'review_packet.json');draft=c.read(folder/'case_draft.json');root=c.case_dir(case_id)/c.STAGES[4]/'runs'/packet['closed_loop_run_id'];construction=c.case_dir(case_id)/c.STAGES[3]/'runs'/packet['construction_run_id'];config=c.read(construction/'case_config.json');cert=c.read(construction/'reach_certificate.json');esc=html.escape
    def img(path,alt):return f'<img alt="{esc(alt)}" src="data:image/png;base64,{base64.b64encode(path.read_bytes()).decode()}">'
    status=c.read(folder/'accepted_review.json')['decision'] if (folder/'accepted_review.json').exists() else 'pending'
    checklabels=['我已观看闭环视频','接受保守范围证书作为起点失败依据（未执行伪造失败动作）','接受仅改变机器人初始 base 的修改清单','接受“同侧平移可解决”分类，不把它当必须换边','接受本次三次全成功门槛，资产稳定性门槛不改']
    fields=''.join(f'<label><input id="{key}" type="checkbox">{esc(label)}</label>' for key,label in zip(CHECKS,checklabels))
    sha=c.digest(folder/'review_packet.json');video=f'/evidence/{str((root/"delivery/case1_r_closed_loop.mp4").relative_to(c.WORKSPACE))}'
    physical=c.read(root/'summary.json');rows=''.join(f'<tr><td>{r["trial"]}</td><td>{esc(r["status"])}</td><td>{r["total_physical_time_s"]:.2f} s</td><td>到站无重置</td></tr>' for r in physical['trials'])
    checks_json=json.dumps(CHECKS)
    doc=f'''<!doctype html><html lang="zh"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>最终人工审阅 {esc(case_id)}</title><style>body{{font:17px/1.7 system-ui;background:#f5f7f9;color:#223;max-width:1150px;margin:25px auto;padding:18px}}section{{background:white;border-radius:12px;padding:24px;margin:18px 0}}img,video{{max-width:100%;height:auto}}label{{display:block;margin:12px 0}}input:not([type=checkbox]),textarea{{width:100%;box-sizing:border-box;font:inherit}}button{{font:inherit;margin:8px;padding:8px 16px}}code,pre{{white-space:pre-wrap;overflow-wrap:anywhere}}table{{border-collapse:collapse;width:100%}}td,th{{padding:8px;text-align:left;border-bottom:1px solid #ddd}}.warn{{border-left:5px solid #d69431}}</style><h1>{esc(case_id)} · 步骤 6：最终人工审阅</h1><section class="warn"><b>物理 3 / 3 严格通过；人工状态：{esc(status)}。</b><p>本次按用户明确要求采用三次全成功，覆盖方案默认完整链路五次要求，未修改物理阈值或资产名义/扰动门槛。只有人工接受后才能正式导出。Agent 不能代填批准，人工不能覆盖物理失败。</p><p><b>分类：Case 1-R · 同侧平移可解决。</b>不声称必须换边或西侧所有站位都失败。</p></section><section><h2>起点、终点与修改清单</h2><p>P131：<code>{esc(str(config['start_base']))}</code> → A：<code>{esc(str(config['goal_base']))}</code>。目标、家具和资产不变，仅改变初始底盘。</p><p>起点距目标 {cert['start_base_to_target_xy_m']:.6f} m &gt; 保守接触上界 {cert['contact_radius_m']:.6f} m；覆盖双臂、全原地朝向与连续合法躯干。目标可见 {config['target_visible_pixels']} 像素。</p>{img(construction/config['views_dir']/'initial_robot_view.png','困难起点 head_camera 视角')}</section><section><h2>完整链路视频与成功视角</h2><video id="video" controls preload="metadata" poster="data:image/png;base64,{base64.b64encode((root/'delivery/preview_hold.png').read_bytes()).decode()}"><source src="{video}" type="video/mp4"></video><p id="offline">在线服务可直接播放。离线打开时，请点击下面的原始视频链接查看。</p><p><a href="../../../../05_closed_loop/runs/{packet['closed_loop_run_id']}/delivery/case1_r_closed_loop.mp4">原始视频</a>（导航 8×、躯干调整 4×、抓取 1×，墙体仅视觉剖切）。</p>{img(root/'delivery/closed_loop_evidence.png','规划路径与实测轨迹、抬升证据')}<table><tr><th>试验</th><th>严格验收</th><th>总物理时间</th><th>连续性</th></tr>{rows}</table></section><section><h2>原始粗站位图</h2>{img(c.case_dir(case_id)/c.STAGES[2]/'runs/coarse_v1/station_map.png','步骤 3 原始站位图')}<p>其他配置的执行失败、搜索耗尽与保守范围超出分开标注。当前 P131 的强失败证据是几何范围证书。</p></section><section><h2>你的最终决定</h2><p>审阅包 SHA-256：<code>{sha}</code>。新构造/新证据需新审阅，不覆盖本次记录。</p><label>审阅者<input id="reviewer" placeholder="填写你的名字"></label>{fields}<label>备注<textarea id="notes"></textarea></label><button onclick="submitDecision('accepted')">接受</button><button onclick="submitDecision('reconstruct')">要求重新构造</button><button onclick="submitDecision('rejected')">拒绝</button><button onclick="downloadDecision()">下载决定 JSON</button><select id="decision"><option value="accepted">accepted</option><option value="reconstruct">reconstruct</option><option value="rejected">rejected</option></select><pre id="result"></pre><p>也可编辑本目录 human_review.json，再运行 accept-final。accepted 仍会自动检查全部物理证据和哈希。</p></section><script>const checks={checks_json};function data(decision){{let d={{case_id:{json.dumps(case_id)},review_id:{json.dumps(review_id)},review_packet_sha256:{json.dumps(sha)},actor:'human',decision,reviewer:document.getElementById('reviewer').value,notes:document.getElementById('notes').value}};for(let k of checks)d[k]=document.getElementById(k).checked;return d}}async function submitDecision(decision){{try{{if(location.protocol==='file:')throw Error('离线请下载 JSON 后用 accept-final 导入');let r=await fetch('/api/final-review',{{method:'POST',headers:{{'Content-Type':'application/json'}},body:JSON.stringify(data(decision))}});let result=await r.json();document.getElementById('result').textContent=JSON.stringify(result,null,2)}}catch(e){{document.getElementById('result').textContent=e.message}}}}function downloadDecision(){{let d=data(document.getElementById('decision').value),url=URL.createObjectURL(new Blob([JSON.stringify(d,null,2)],{{type:'application/json'}}));let a=document.createElement('a');a.href=url;a.download={json.dumps(case_id+'-final-review.json')};a.click();URL.revokeObjectURL(url)}}if(location.protocol==='file:')document.getElementById('video').querySelector('source').src='../../../../05_closed_loop/runs/{packet['closed_loop_run_id']}/delivery/case1_r_closed_loop.mp4';</script></html>'''
    # reviews/<id> → case 的相对路径为 ../../../，不是 ../../../../。
    doc=doc.replace('../../../../05_closed_loop/','../../../05_closed_loop/')
    (folder/'review.html').write_text(doc)

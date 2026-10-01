#!/usr/bin/env python3
"""从原始轨迹重判，不采信报告中的 success 字段。"""
from pathlib import Path
import json
from lastmile.audit import sha256,write_json
from lastmile.admission import admit_asset
from lastmile.validation import validate_motion_prefix
W=Path(__file__).absolute().parents[1];P=json.loads((W/'configs/protocol_v1.json').read_text());shared=W/'configs/cup_admission_v1.json';manifest=json.loads(shared.read_text());hash_=sha256(shared)
paths=[W/'reports/E013-cup-admission-nominal.json',W/'reports/E012-cup-admission-perturbed.json']
reports=[json.loads(p.read_text()) if p.exists() else {'trials':[]} for p in paths];rows=[t for r in reports for t in r['trials']]
if len(rows)!=25:
    verdict={'asset_id':'Cup_30','status':'pending_physics','reason':'insufficient_independent_trials','nominal_completed':len(reports[0]['trials']),'perturbed_completed':len(reports[1]['trials'])}
else:
    trials=[]
    source=W/'runs/stage_a/E005-controller-calibration/0014_seed0/left_h0.7380/grasp_0794_trace.json';source_initial=json.loads(source.read_text())['initial']
    for i,row in enumerate(rows):
        if row['selection_protocol_sha256']!=hash_:raise ValueError('试验选择协议不一致')
        path=Path(row['trace_path']) if row.get('trace_path') else None
        if row['kind']=='nominal':
            trace=json.loads(path.read_text());prefix=trace['prefix_samples'];prefix_initial=trace['prefix_initial']
        else:
            j=i-5
            if row['base_delta']!=manifest['trials'][j]['base_delta']:raise ValueError('扰动与预注册不符')
            directory=W/f'runs/repeats/E012-cup-admission-perturbed/perturbed_{j:02d}'
            pfx=json.loads((directory/'torso_adjust_trace.json').read_text());prefix=pfx['samples'];prefix_initial=pfx['initial']
            trace=json.loads(path.read_text()) if path and path.exists() else {'initial':{**source_initial,'base':prefix_initial['base'],'head':prefix_initial['head'],'idle_arm':prefix_initial['idle_arm']},'samples':prefix}
        if validate_motion_prefix(prefix,prefix_initial,P['thresholds']):
            # 保留失败的完整轨迹，准入时不得转成成功。
            trace['samples'][0]['illegal'].append('prefix_protocol_failure')
        trials.append({**row,'initial':trace['initial'],'trace':trace['samples']})
    verdict=admit_asset('Cup_30','ordinary',trials,sha256(W/'configs/protocol_v1.json'),P)
    verdict.update(grasp_ids=[794],source_index=14,side='left',h=.738,selection_protocol_sha256=hash_,evidence_reports=[str(p) for p in paths],scope='only this asset/grasp and tested station error distribution; not other scenes or navigation')
asset=json.loads((W/'registry/assets/Cup_30.json').read_text());episode=json.loads((W/'registry/episodes/0014.json').read_text())
verdict.update(asset_xml_sha256=asset['asset_xml_sha256'],grasp_sha256=asset['grasp_sha256'],benchmark_sha256=episode['provenance']['benchmark_sha256'],episode_sha256=sha256(W/'registry/episodes/0014.json'),compiled_model_sha256=sha256(W/'runs/stage_a/E002-corrected/0014_seed0/compiled_model.mjb'))
write_json(W/'reports/cup_admission.json',verdict)
if verdict['status']=='admitted':
    white=json.loads((W/'registry/whitelist.json').read_text());white['status']='physical_admissions_present';white['assets']=[a for a in white.get('assets',[]) if a.get('asset_id')!='Cup_30']+[verdict];write_json(W/'registry/whitelist.json',white)
    asset_path=W/'registry/assets/Cup_30.json';asset=json.loads(asset_path.read_text());asset.update(admission_status='admitted',allowed_grasp_ids=[794],admission_evidence=verdict);write_json(asset_path,asset)
print(json.dumps(verdict,ensure_ascii=False,indent=2))

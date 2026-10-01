#!/usr/bin/env python3
from pathlib import Path
import json
from lastmile.audit import write_json,sha256
from lastmile.validation import validate_trace,validate_motion_prefix
W=Path(__file__).absolute().parents[1];P=json.loads((W/'configs/protocol_v1.json').read_text());positives=[]
for index,name,side,gid in [(14,'E009-cup-force-aware','left',794),(1949,'E010-candle-force-aware','right',600)]:
 path=W/f'reports/{name}.json';report=json.loads(path.read_text()) if path.exists() else {'trials':[]};verdicts=[]
 for row in report['trials']:
  trace=json.loads(Path(row['trace_path']).read_text());prefix=validate_motion_prefix(trace['prefix_samples'],trace['prefix_initial'],P['thresholds'])
  verdict=validate_trace(trace['samples'],trace['initial'],P['thresholds']);verdicts.append({'trial_id':row['trial_id'],'prefix_failure':prefix,**verdict})
 success=sum(v['status']=='verified_success' and not v['prefix_failure'] for v in verdicts)
 positives.append({'source_index':index,'side':side,'h':.738,'grasp_id':gid,'nominal_completed':len(verdicts),'nominal_successes':success,'evidence_report':str(path),'status':'passed' if len(verdicts)==5 and success==5 else 'pending','verdicts':verdicts})
result={'stage':'A','status':'passed' if all(p['status']=='passed' for p in positives) else 'pending','protocol_sha256':sha256(W/'configs/protocol_v1.json'),'contact_sampler_version':'force-aware-v2','positive_regressions':positives,'scope':'20D and single-arm configuration/static calibration plus two original positive scenes; nominal fixed-controller physical repeatability, not randomized planner success or asset whitelist','finalization_code_sha256':{p.name:sha256(p) for p in (W/'src/lastmile').glob('*.py')}}
write_json(W/'registry/protocol_calibration.json',result);write_json(W/'reports/stage_a_verdict.json',result);print(json.dumps({k:v for k,v in result.items() if k!='finalization_code_sha256'},ensure_ascii=False,indent=2))

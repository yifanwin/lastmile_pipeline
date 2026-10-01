#!/usr/bin/env python3
"""独立恢复完整场景后重执行冻结的控制序列；不是复制旧 verdict。"""
import argparse,json
from pathlib import Path
import mujoco
from lastmile.simulation import Simulation
from lastmile.validation import validate_trace,validate_motion_prefix
from lastmile.audit import sha256,write_json
p=argparse.ArgumentParser();p.add_argument('--index',type=int,required=True);p.add_argument('--cache-dir',type=Path,required=True)
p.add_argument('--source-condition',type=Path,required=True);p.add_argument('--grasp-id',type=int,required=True);p.add_argument('--side',choices=['left','right'],required=True)
p.add_argument('--count',type=int,default=5);p.add_argument('--run-id',required=True)
p.add_argument('--selection-protocol',type=Path)
a=p.parse_args();W=Path(__file__).absolute().parents[1];root=W/'runs/repeats'/a.run_id
if root.exists():raise ValueError('不覆盖历史运行')
source=a.source_condition/f'grasp_{a.grasp_id:04d}_trace.json';prefix=a.source_condition/'torso_adjust_trace.json'
old=json.loads(source.read_text());adjust=json.loads(prefix.read_text());P=json.loads((W/'configs/protocol_v1.json').read_text())
selection={'method':'frozen control sequence; fresh independent full-state restore and physical execution','source_trace_sha256':sha256(source),'source_prefix_sha256':sha256(prefix),'grasp_id':a.grasp_id,'source_index':a.index,'side':a.side}
write_json(root/'selection_protocol.json',selection);selection_hash=sha256(root/'selection_protocol.json')
if a.selection_protocol:
    shared=json.loads(a.selection_protocol.read_text())
    if (shared['source_index'],shared['side'],shared['grasp_ids'])!=(a.index,a.side,[a.grasp_id]):raise ValueError('冻结选择协议错配')
    selection_hash=sha256(a.selection_protocol)
    write_json(root/'shared_selection_protocol.json',shared)
if a.selection_protocol:
    n=shared['nominal_controller']
    if n['source_trace_sha256']!=sha256(source) or n['source_prefix_sha256']!=sha256(prefix) or shared['nominal_trials']!=a.count:raise ValueError('冻结名义控制器错配')
sim=Simulation.from_cached(W,a.index,a.cache_dir,root)
results=[]
for trial in range(a.count):
    sim.restore();new_prefix=[]
    for row in adjust['samples']:
        sim.data.ctrl[:]=row['ctrl'];mujoco.mj_step(sim.model,sim.data);new_prefix.append(sim.sample(a.side,row['h']))
    prefix_error=validate_motion_prefix(new_prefix,adjust['initial'],P['thresholds'])
    samples=[]
    for row in old['samples']:
        sim.data.ctrl[:]=row['ctrl'];mujoco.mj_step(sim.model,sim.data);samples.append(sim.sample(a.side,row['h']))
    initial={**old['initial'],'base':new_prefix[-1]['base'],'head':new_prefix[-1]['head'],'idle_arm':new_prefix[-1]['idle_arm']}
    verdict=validate_trace(samples,initial,P['thresholds']) if not prefix_error else {'status':'executed_failure','reason':prefix_error}
    destination=root/f'nominal_{trial:02d}.json';write_json(destination,{'initial':initial,'samples':samples,'prefix_initial':adjust['initial'],'prefix_samples':new_prefix})
    record={'trial_id':f'{a.run_id}-nominal-{trial:02d}','kind':'nominal','snapshot_restored':True,'protocol_sha256':sha256(W/'configs/protocol_v1.json'),'selection_protocol_sha256':selection_hash,'trace_path':str(destination),'prefix_checked':True,**verdict}
    results.append(record);write_json(W/f'reports/{a.run_id}.json',{'selection':selection,'trials':results,'successes':sum(r['status']=='verified_success' for r in results),'note':'deterministic controller repeatability only; not perturbed or fresh planning success rate'})
    print(json.dumps(record,ensure_ascii=False),flush=True)

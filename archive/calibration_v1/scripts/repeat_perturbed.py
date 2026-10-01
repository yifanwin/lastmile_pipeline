#!/usr/bin/env python3
"""冻结单个合规 grasp，在预注册的到站误差中重新规划并真实执行。"""
import argparse,json,copy
from pathlib import Path
import mujoco
import numpy as np
from lastmile.simulation import Simulation
from lastmile.calibration_runner import run_one
from lastmile.audit import sha256,write_json
p=argparse.ArgumentParser();p.add_argument('--index',type=int,required=True);p.add_argument('--cache-dir',type=Path,required=True)
p.add_argument('--side',choices=['left','right'],required=True);p.add_argument('--h',type=float,required=True);p.add_argument('--grasp-id',type=int,required=True)
p.add_argument('--count',type=int,default=20);p.add_argument('--run-id',required=True)
p.add_argument('--selection-protocol',type=Path)
a=p.parse_args();W=Path(__file__).absolute().parents[1];root=W/'runs/repeats'/a.run_id
if root.exists():raise ValueError('不覆盖历史运行')
P=json.loads((W/'configs/protocol_v1.json').read_text());rng=np.random.default_rng(20261001)
deltas=rng.uniform(-1,1,(a.count,3))*[P['perturbations']['base_xy_m'],P['perturbations']['base_xy_m'],P['perturbations']['base_yaw_rad']]
selection={'source_index':a.index,'side':a.side,'h':a.h,'grasp_ids':[a.grasp_id],'world_geometry':'mesh','time_dilation_factor':.25,'approach_overshoot_m':.01,'error_distribution':'independent uniform world dx,dy,yaw; fixed RNG 20261001','trials':[{'seed':i+10,'base_delta':d.tolist()} for i,d in enumerate(deltas)],'selection':'one frozen source NPZ row; no fallback or reselection'}
write_json(root/'selection_protocol.json',selection);selection_hash=sha256(root/'selection_protocol.json')
if a.selection_protocol:
    shared=json.loads(a.selection_protocol.read_text())
    if (shared['source_index'],shared['side'],shared['grasp_ids'])!=(a.index,a.side,[a.grasp_id]):raise ValueError('冻结选择协议错配')
    selection_hash=sha256(a.selection_protocol)
    write_json(root/'shared_selection_protocol.json',shared)
if a.selection_protocol:
    if len(shared['trials'])!=a.count or shared['h']!=a.h:raise ValueError('预注册试验数或 h 错配')
    deltas=np.array([t['base_delta'] for t in shared['trials']])
sim=Simulation.from_cached(W,a.index,a.cache_dir,root);original_state=sim.initial_state.copy();original_episode=copy.deepcopy(sim.episode);results=[]
for i,delta in enumerate(deltas):
    sim.initial_state=original_state.copy();sim.episode=copy.deepcopy(original_episode);sim.restore()
    base=sim.group('base')+delta
    for name,q in zip(['base_x','base_y','base_theta'],base):sim.data.qpos[sim.qpos_adr[name]]=q
    sim.episode['robot']['init_qpos']['base']=base.tolist();mujoco.mj_forward(sim.model,sim.data);sim.hold_ctrl()
    mujoco.mj_getState(sim.model,sim.data,sim.initial_state,sim.state_spec)
    dest=root/f'perturbed_{i:02d}';dest.mkdir();np.savez_compressed(dest/'initial_snapshot.npz',state=sim.initial_state)
    verdict=run_one(W,a.index,a.side,a.h,i+10,dest,1,sim=sim,world_geometry='mesh',time_dilation=.25,approach_overshoot=.01,grasp_ids=[a.grasp_id])
    trace=dest/f'grasp_{a.grasp_id:04d}_trace.json'
    record={'trial_id':f'{a.run_id}-perturbed-{i:02d}','kind':'perturbed','snapshot_restored':True,'protocol_sha256':sha256(W/'configs/protocol_v1.json'),'selection_protocol_sha256':selection_hash,'trace_path':str(trace) if trace.exists() else None,'base_delta':delta.tolist(),**verdict}
    results.append(record);write_json(W/f'reports/{a.run_id}.json',{'selection':selection,'trials':results,'successes':sum(r['status']=='verified_success' for r in results),'planned_trials':a.count})
    print(json.dumps(record,ensure_ascii=False),flush=True)

#!/usr/bin/env python3
import argparse,json
from pathlib import Path
from lastmile.calibration_runner import run_one
from lastmile.simulation import Simulation
from lastmile.audit import write_json
p=argparse.ArgumentParser()
p.add_argument('--index',type=int,required=True);p.add_argument('--arms',nargs='+',choices=['left','right'],default=['left','right'])
p.add_argument('--heights',nargs='+',type=float,default=[0.,.1845,.369,.5535,.738]);p.add_argument('--seed',type=int,default=0)
p.add_argument('--max-grasps',type=int,default=24)
p.add_argument('--run-id',default='E002-corrected')
p.add_argument('--cache-dir',type=Path)
p.add_argument('--world-geometry',choices=['aabb','mesh'],default='aabb')
p.add_argument('--time-dilation',type=float,default=1.)
p.add_argument('--approach-overshoot',type=float,default=0.)
a=p.parse_args();W=Path(__file__).absolute().parents[1]
run_dir=W/f'runs/stage_a/{a.run_id}/{a.index:04d}_seed{a.seed}'
if run_dir.exists():raise ValueError('运行目录已存在，请指定新的 --run-id；不覆盖历史证据')
sim=Simulation.from_cached(W,a.index,a.cache_dir,run_dir) if a.cache_dir else Simulation(W,a.index,run_dir)
results=[]
for h in a.heights:
    for side in a.arms:
        dest=run_dir/f'{side}_h{h:.4f}'
        result=run_one(W,a.index,side,h,a.seed,dest,a.max_grasps,sim=sim,world_geometry=a.world_geometry,time_dilation=a.time_dilation,approach_overshoot=a.approach_overshoot)
        results.append(result);print(json.dumps(result,ensure_ascii=False),flush=True)
        write_json(W/f'reports/{a.run_id}_positive_{a.index:04d}_seed{a.seed}.json',{'results':results})

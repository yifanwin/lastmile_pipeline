#!/usr/bin/env python3
"""不再调用规划器：从校准快照重放真实逐物理步 ctrl。"""
import argparse,json
from pathlib import Path
import mujoco
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--model',type=Path,required=True);p.add_argument('--snapshot',type=Path,required=True);p.add_argument('--trace',type=Path,required=True)
a=p.parse_args();m=mujoco.MjModel.from_binary_path(str(a.model));d=mujoco.MjData(m)
state=np.load(a.snapshot)['state'];mujoco.mj_setState(m,d,state,mujoco.mjtState.mjSTATE_INTEGRATION);mujoco.mj_forward(m,d)
trace=json.loads(a.trace.read_text());target=m.body(trace['initial']['target_id']).id
errors=[]
for row in trace['samples']:
    if 'ctrl' not in row:raise ValueError('旧版诊断轨迹未记录 ctrl，不能声称可重放')
    ctrl=np.array(row['ctrl']);
    if ctrl.shape!=(m.nu,):raise ValueError('模型与 ctrl 维度错配')
    d.ctrl[:]=ctrl;mujoco.mj_step(m,d)
    errors.append(abs(float(d.xpos[target][2])-row['height_m']))
print('samples',len(errors),'max_height_error_m',max(errors),'not_a_fresh_success_verdict')

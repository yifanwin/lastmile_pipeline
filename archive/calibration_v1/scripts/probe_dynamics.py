#!/usr/bin/env python3
"""实际机器人动力学协议探针，不替代正例抓取验收。"""
from pathlib import Path
import json
import mujoco
import numpy as np
from lastmile.simulation import Simulation
from lastmile.protocol import torso_joints,waypoint_action
from lastmile.audit import write_json
W=Path(__file__).absolute().parents[1];A=W.parent/'molmospaces_data/assets'
P=json.loads((W/'configs/protocol_v1.json').read_text())
spec=mujoco.MjSpec.from_file(str(A/'robots/rby1m/rby1_v1.2_site_control.xml'))
for b in spec.bodies:
    if b.name.startswith('robot_0/'):b.gravcomp=1
spec.option.timestep=.004
sim=Simulation.__new__(Simulation);sim.model=spec.compile();sim.data=mujoco.MjData(sim.model)
m,d=sim.model,sim.data
sim.qpos_adr={m.joint(i).name.removeprefix('robot_0/'):int(m.jnt_qposadr[i]) for i in range(m.njnt) if m.joint(i).name.startswith('robot_0/')}
sim.act={m.actuator(i).name.removeprefix('robot_0/'):i for i in range(m.nu) if m.actuator(i).name.startswith('robot_0/')}
for side in ('left','right'):
    for i,q in enumerate([.5,0,0,-2.3,0,-.5,0]):d.qpos[sim.qpos_adr[f'{side}_arm_{i}']]=q
    d.qpos[sim.qpos_adr[f'gripper_finger_{side[0]}1']]=-.05
    d.qpos[sim.qpos_adr[f'gripper_finger_{side[0]}2']]=.05
mujoco.mj_forward(m,d);sim.hold_ctrl();initial=d.qpos.copy()
rows=[]
for h in P['h_candidates']:
    mujoco.mj_resetData(m,d);d.qpos[:]=initial;mujoco.mj_forward(m,d);sim.hold_ctrl()
    max_error=0.;max_base_drift=0.;max_linkage=0.
    for step in range(max(1,int(h/.001)+1)):
        command=min(h,step*.001)
        sim.action(waypoint_action(sim.group('left_arm'),sim.group('left_arm'),'left',command),phase='torso_adjust')
        for _ in range(10):
            mujoco.mj_step(m,d)
            max_error=max(max_error,float(np.max(np.abs(sim.group('torso')-torso_joints(command)))))
            max_base_drift=max(max_base_drift,float(np.linalg.norm(sim.group('base')[:2])))
            tq=sim.group('torso');max_linkage=max(max_linkage,float(np.max(np.abs(tq-np.array([0.,tq[1],-2*tq[1],tq[1],0.,0.])))))
    for _ in range(250):mujoco.mj_step(m,d)
    final=float(np.max(np.abs(sim.group('torso')-torso_joints(h))))
    record={'h':h,'max_command_tracking_error_rad':max_error,'final_command_tracking_error_rad':final,
            'max_base_translation_drift_m':max_base_drift,
            'max_linkage_error_rad':max_linkage,'final_linkage_error_rad':float(np.max(np.abs(sim.group('torso')-np.array([0.,sim.group('torso')[1],-2*sim.group('torso')[1],sim.group('torso')[1],0.,0.])))),
            'scope':'robot-only dynamics; not scene safety or Pick validation'}
    rows.append(record);print(record,flush=True)
write_json(W/'reports/dynamics_probe.json',{'results':rows,'physics_timestep_s':.004})

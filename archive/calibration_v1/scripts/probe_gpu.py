#!/usr/bin/env python3
import json
from pathlib import Path
import numpy as np
import torch
from lastmile.audit import write_json
from molmo_spaces.planner.curobo_planner import CuroboPlanner,CuroboPlannerConfig
W=Path(__file__).absolute().parents[1]
print('cuda',torch.cuda.is_available(),torch.cuda.get_device_name(0),flush=True)
rows=[]
for side in ['left','right']:
    path=W/f'registry/planner_configs/{side}_h0.0000.yml'
    planner=CuroboPlanner(CuroboPlannerConfig(curobo_robot_config_path=str(path),num_ik_seeds=64,num_trajopt_seeds=4,max_attempts=5,collision_activation_distance=.01))
    print('active joints',planner.joint_names,flush=True)
    pose=planner.fk_solve([.5,0,0,-2.3,0,-.5,0])
    rows.append({'side':side,'active_joints':planner.joint_names,'fk_pose':pose.tolist()})
    del planner
    torch.cuda.empty_cache()
write_json(W/'reports/gpu_planner_probe.json',{'device':torch.cuda.get_device_name(0),'results':rows})

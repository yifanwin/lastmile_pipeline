#!/usr/bin/env python3
"""无解诊断对照，不执行抓取、不计入成功率：环境/自碰撞/运动学。"""
import argparse,json,gc
from pathlib import Path
import numpy as np
import mujoco
import torch
import yaml
from curobo.geom.types import WorldConfig
from molmo_spaces.planner.curobo_planner import CuroboPlanner,CuroboPlannerConfig
from lastmile.simulation import Simulation,body_pose,pose7
from lastmile.calibration_runner import world_obstacles
from lastmile.protocol import arm_only_config,to_base_frame,torso_joints
from lastmile.audit import write_json
p=argparse.ArgumentParser();p.add_argument('--index',type=int,required=True);p.add_argument('--cache-dir',type=Path,required=True);p.add_argument('--h',type=float,default=.738);a=p.parse_args()
W=Path(__file__).absolute().parents[1]
sim=Simulation.from_cached(W,a.index,a.cache_dir,W/f'runs/diagnose_ik/{a.index:04d}')
for i,q in enumerate(torso_joints(a.h)):sim.data.qpos[sim.qpos_adr[f'torso_{i}']]=q
mujoco.mj_forward(sim.model,sim.data)
uid=sim.episode['provenance']['asset_id'];record=json.loads((W/f'registry/assets/{uid}.json').read_text())
with np.load(record['grasp_path']) as z:grasps=z['transforms'].astype(float)
u,_,v=np.linalg.svd(grasps[:,:3,:3]);grasps[:,:3,:3]=u@v
world=body_pose(sim.data,sim.target)@grasps
base=body_pose(sim.data,sim.model.body('robot_0/base').id)
rows=[]
for side in ['left','right']:
    candidates=np.argsort(np.linalg.norm(world[:,:3,3]-sim.tcp(side)[:3,3],axis=1),kind='stable')[:24]
    poses=[pose7(to_base_frame(world[g],base)) for g in candidates]
    idle='right' if side=='left' else 'left';root=sim.assets/'robots/rby1m/curobo_config'
    template=yaml.safe_load((root/'rby1m_holobase.yml').read_text())
    cfg=arm_only_config(template,side,a.h,sim.group(idle+'_arm'),sim.group('head'))
    kin=cfg['robot_cfg']['kinematics'];kin.update(urdf_path=str(root/'urdf/model_holobase.urdf'),asset_root_path=str(root/'urdf/meshes'),collision_spheres=str(root/'rby1m_holobase_spheres.yml'),usd_robot_root=str(root))
    path=sim.run_dir/f'{side}.yml';path.write_text(yaml.safe_dump(cfg,sort_keys=False))
    for name,env,disabled in [('environment_and_self',world_obstacles(sim),False),('self_only',WorldConfig(),False),('kinematics_only',WorldConfig(),True)]:
        torch.manual_seed(0)
        planner=CuroboPlanner(CuroboPlannerConfig(curobo_robot_config_path=str(path),world_config=env,collision_cache={'mesh':3,'obb':512},num_ik_seeds=64,num_trajopt_seeds=4,max_attempts=5,collision_activation_distance=.01))
        mask=planner.ik_solve_batch(poses,seed_config=sim.group(side+'_arm').tolist(),num_seeds=64,disable_collision=disabled,pad_to=24)
        row={'side':side,'h':a.h,'condition':name,'checked':24,'solutions':int(mask.sum()),'grasp_ids':candidates.tolist(),'solved_grasp_ids':candidates[mask].tolist(),'evidence':'IK only; no physical success or reachability proof'}
        rows.append(row);print(json.dumps(row),flush=True)
        del planner;gc.collect();torch.cuda.empty_cache()
        write_json(W/f'reports/diagnose_ik_{a.index:04d}.json',{'results':rows,'state':'kinematic h set for diagnosis only, not a safe torso trajectory'})

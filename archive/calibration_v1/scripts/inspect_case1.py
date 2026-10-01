from pathlib import Path
import json
import numpy as np
import mujoco
from PIL import Image
from lastmile.simulation import Simulation,body_pose
from molmo_spaces.utils.mj_model_and_data_utils import body_aabb
W=Path(__file__).absolute().parents[1];out=W/'runs/case1/E015-explore'
sim=Simulation.from_cached(W,14,W/'runs/stage_a/E002-corrected/0014_seed0',out)
m,d=sim.model,sim.data;table=sim.episode['provenance']['support_parent'];tid=m.body(table).id
center,size=body_aabb(m,d,tid,visible_only=False)
print(json.dumps({'table':table,'center':center.tolist(),'size':size.tolist(),'target':d.xpos[sim.target].tolist(),'base':sim.group('base').tolist(),'table_pose':body_pose(d,tid).tolist()},indent=2),flush=True)
m.vis.global_.offwidth=1280;m.vis.global_.offheight=960
r=mujoco.Renderer(m,height=960,width=1280)
for label,az,el,dist in [('overhead',0,-89,5),('oblique',110,-35,3.5),('opposite',-70,-30,3.5)]:
 c=mujoco.MjvCamera();c.lookat[:]=center;c.lookat[2]=.6;c.azimuth=az;c.elevation=el;c.distance=dist
 r.update_scene(d,camera=c);Image.fromarray(r.render()).save(out/f'{label}.png')
r.close()

"""生成 Case 1-R 的保守工作空间证据和绕桌路线；不生成成功标签。"""
from pathlib import Path
import json
import numpy as np
import mujoco
from lastmile.simulation import Simulation,body_pose
from lastmile.navigation import build_grid
from lastmile.audit import write_json,sha256
from molmo_spaces.utils.mj_model_and_data_utils import body_aabb
W=Path(__file__).absolute().parents[1];out=W/'runs/case1/E016-preflight';out.mkdir(parents=True,exist_ok=True)
sim=Simulation.from_cached(W,14,W/'runs/stage_a/E002-corrected/0014_seed0',out);m,d=sim.model,sim.data
center,size=body_aabb(m,d,m.body(sim.episode['provenance']['support_parent']).id,visible_only=False)
start=np.array([7.78,.43,np.pi/2]);goal=sim.group('base').copy()
# 相同两根 z=0.35 link 和 y 轴，h 与 -h 的水平分量严格抵消。
for name in ['link_torso_2','link_torso_3']:
    if not np.allclose(m.body('robot_0/'+name).pos,[0,0,.35],atol=1e-7):raise ValueError('联动解析前提不满足')
for i in [1,2,3]:
    if not np.allclose(m.joint('robot_0/torso_'+str(i)).axis,[0,1,0]):raise ValueError('联动轴解析前提不满足')
rows=[]
for side in ['left','right']:
    shoulder=m.body(f'robot_0/link_{side}_arm_0').id;radius=0.
    for fid in [m.body(f'robot_0/ee_finger_{side[0]}{i}').id for i in [1,2]]:
        path=[];b=fid
        while b!=shoulder:
            if b<=0:raise ValueError('手指不在肩部链中')
            path.append(b);b=int(m.body_parentid[b])
        chain=sum(np.linalg.norm(m.body_pos[b]) for b in path)
        for b in path+[shoulder]:
            for j in range(int(m.body_jntadr[b]),int(m.body_jntadr[b])+int(m.body_jntnum[b])):
                if m.jnt_type[j]==mujoco.mjtJoint.mjJNT_SLIDE:chain+=max(abs(m.jnt_range[j]))
                else:chain+=2*np.linalg.norm(m.jnt_pos[j])
        for g in range(int(m.body_geomadr[fid]),int(m.body_geomadr[fid])+int(m.body_geomnum[fid])):
            if m.geom_contype[g] or m.geom_conaffinity[g]:radius=max(radius,chain+np.linalg.norm(m.geom_pos[g])+m.geom_rbound[g])
    rows.append({'side':side,'shoulder_horizontal_radius_m':float(np.linalg.norm(m.body_pos[shoulder][:2])),'shoulder_to_any_finger_collision_point_bound_m':float(radius)})
target_radius=0.
root_pose=body_pose(d,sim.target)
for g in range(m.ngeom):
    if int(m.body_rootid[m.geom_bodyid[g]])==sim.target_root and (m.geom_contype[g] or m.geom_conaffinity[g]):
        target_radius=max(target_radius,np.linalg.norm(d.geom_xpos[g]-root_pose[:3,3])+m.geom_rbound[g])
max_contact=max(r['shoulder_horizontal_radius_m']+r['shoulder_to_any_finger_collision_point_bound_m'] for r in rows)+target_radius+.03
actual_distance=np.linalg.norm(start[:2]-d.xpos[sim.target,:2]);south_band_y=center[1]-size[1]/2-.30
same_side_lower=d.xpos[sim.target,1]-south_band_y
if actual_distance<=max_contact or same_side_lower<=max_contact:raise ValueError('起点没有可达范围分离余量')
certificate={'method':'triangle-inequality bound to every physical selected-finger collision point; torso horizontal cancellation for [0,h,-2h,h,0,0]','coverage':'both arms, all arm configurations, all yaw, continuous h in [0,.738], all target collision geometry; not an IK budget inference','arm_bounds':rows,'target_collision_radius_bound_m':float(target_radius),'tracking_and_settling_inflation_m':.03,'base_to_target_contact_radius_bound_m':float(max_contact),'start_base_to_target_xy_m':float(actual_distance),'gap_m':float(actual_distance-max_contact),'same_south_side_band_y_max':float(south_band_y),'same_south_side_min_target_distance_m':float(same_side_lower),'same_side_gap_m':float(same_side_lower-max_contact),'scope':'south operation band only; does not claim east/west sides inaccessible','robot_model_sha256':sha256(sim.assets/'robots/rby1m/rby1_v1.2_site_control.xml')}
write_json(W/'reports/case1_reach_certificate.json',certificate)
grid=build_grid(sim,start,goal,center,size,radius=.42)
np.savez_compressed(out/'navigation_grid.npz',low=grid['low'],spacing=grid['spacing'],free=grid['free'],floors=grid['floors'],path=grid['path'])
config={'case_id':'case1-r-001','source_index':14,'asset_id':'Cup_30','target_name':sim.target_name,'support_parent':sim.episode['provenance']['support_parent'],'case_type':'Case 1-R; south-side contact unreachable, navigate around west edge to north-side witness','start_base':start.tolist(),'goal_base':goal.tolist(),'table_center':center.tolist(),'table_size':size.tolist(),'side':'left','h':.738,'grasp_ids':[794],'nav_path':grid['path'].tolist(),'nav_max_speed_m_s':.025,'nav_acceleration_m_s2':.01,'nav_max_yaw_speed_rad_s':.05,'nav_yaw_acceleration_rad_s2':.02,'nav_grid_spacing_m':grid['spacing'],'nav_grid_inflation_radius_m':grid['radius'],'changes':['robot initial base moved to opposite table side; target and all scene objects unchanged'],'expectation':{'baseline_no_navigation':'cannot establish selected finger contact under continuous h/all yaw/both arms bound','closed_loop':'5/5 full restore -> physical navigation -> replan at actual arrival -> strict Pick','failure_action':'retain failed trace; change route/config only under new run ID; no collision disabling or arrival reset'},'protocol_sha256':sha256(W/'configs/protocol_v1.json'),'source_episode_sha256':sha256(W/'registry/episodes/0014.json'),'reach_certificate':'reports/case1_reach_certificate.json'}
write_json(W/'configs/case1_r_001.json',config);print(json.dumps(certificate,indent=2));print('path',grid['path'].shape,'length',np.linalg.norm(np.diff(grid['path'],axis=0),axis=1).sum(),'camera names',[m.camera(i).name for i in range(m.ncam)])
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,ax=plt.subplots(figsize=(7,7));ax.imshow(grid['free'].T,origin='lower',extent=[grid['low'][0],grid['low'][0]+grid['free'].shape[0]*grid['spacing'],grid['low'][1],grid['low'][1]+grid['free'].shape[1]*grid['spacing']],cmap='gray',alpha=.6)
ax.plot(grid['path'][:,0],grid['path'][:,1],c='#2596be',label='conservative A* route');ax.scatter(*start[:2],c='r',label='south start');ax.scatter(*goal[:2],c='g',label='north goal');ax.scatter(*d.xpos[sim.target,:2],c='#e67e22',marker='*',s=100,label='unchanged target');ax.add_patch(plt.Rectangle(center[:2]-size[:2]/2,*size[:2],fill=False,lw=2));ax.set(xlabel='World x (m)',ylabel='World y (m)',title='Case 1-R route preflight (not execution success)');ax.legend();fig.tight_layout()
for ext in ['png','svg']:fig.savefig(W/f'reports/figures/case1_route_preflight.{ext}',dpi=150)

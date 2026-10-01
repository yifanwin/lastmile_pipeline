"""候选站位及保守范围界。所有几何结果与物理标签分开。"""
import numpy as np

def wrap_yaw(v):return float(np.arctan2(np.sin(v),np.cos(v)))
def candidates(center,size,target,A,sides,config):
 rows=[]
 if config['include_exact_A']:rows.append({'pose_id':'A','side':'north','base':list(A),'is_A':True})
 for side in ['north','south','west','east']:
  if side not in sides:continue
  axis=0 if side in ['north','south'] else 1;normal=1-axis
  lower=center[axis]-size[axis]/2-config['corner_extension_m'];upper=center[axis]+size[axis]/2+config['corner_extension_m']
  values=np.linspace(lower,upper,int(np.ceil((upper-lower)/config['along_spacing_m']))+1)
  sign=1 if side in ['north','east'] else -1
  for offset in config['edge_offsets_m']:
   for along in values:
    xy=np.array(center[:2],float);xy[axis]=along;xy[normal]=center[normal]+sign*(size[normal]/2+offset)
    bearing=np.arctan2(target[1]-xy[1],target[0]-xy[0])
    for yaw in config['yaw_offsets_deg']:
     rows.append({'pose_id':f'P{len(rows):03d}','side':side,'base':[float(xy[0]),float(xy[1]),wrap_yaw(bearing+np.deg2rad(yaw))],'yaw_offset_deg':yaw,'edge_offset_m':offset,'is_A':False})
 return rows

def floor_mask(sim,xs,ys):
 import mujoco
 m,d=sim.model,sim.data;xx,yy=np.meshgrid(xs,ys,indexing='ij');pts=np.c_[xx.ravel(),yy.ravel()];mask=np.zeros(len(pts),bool)
 for g in range(m.ngeom):
  if 'floor' not in m.geom(g).name.lower():continue
  kind=m.geom_type[g]
  if kind==mujoco.mjtGeom.mjGEOM_PLANE:mask[:]=True
  elif kind==mujoco.mjtGeom.mjGEOM_MESH:
   mid=int(m.geom_dataid[g]);va=int(m.mesh_vertadr[mid]);fa=int(m.mesh_faceadr[mid]);vs=m.mesh_vert[va:va+m.mesh_vertnum[mid]]@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g]
   for face in m.mesh_face[fa:fa+m.mesh_facenum[mid]]:
    a,b,c=vs[face,:2];u=b-a;v=c-a;det=u[0]*v[1]-u[1]*v[0]
    if abs(det)<1e-9:continue
    z=pts-a;s=(z[:,0]*v[1]-z[:,1]*v[0])/det;t=(u[0]*z[:,1]-u[1]*z[:,0])/det
    mask|=(s>=-1e-6)&(t>=-1e-6)&(s+t<=1+1e-6)
  elif kind==mujoco.mjtGeom.mjGEOM_BOX:
   local=(np.c_[pts,np.zeros(len(pts))]-d.geom_xpos[g])@d.geom_xmat[g].reshape(3,3)
   mask|=(np.abs(local[:,:2])<=m.geom_size[g,:2]).all(1)
  else:raise ValueError('unsupported floor geometry')
 return mask.reshape(xx.shape)

def place(sim,base,h=0.):
 import mujoco
 from ..protocol import torso_joints
 sim.restore()
 for name,q in zip(['base_x','base_y','base_theta'],base):sim.data.qpos[sim.qpos_adr[name]]=q
 for i,q in enumerate(torso_joints(h)):sim.data.qpos[sim.qpos_adr[f'torso_{i}']]=q
 # 仅机器人初始化自由度清零；保留快照中所有场景对象速度。
 for name in ['base_x','base_y','base_theta']+[f'torso_{i}' for i in range(6)]:
  joint=sim.model.joint('robot_0/'+name).id;sim.data.qvel[int(sim.model.jnt_dofadr[joint])]=0
 sim.hold_ctrl();mujoco.mj_forward(sim.model,sim.data)

def initial_collision(sim):
 m,d=sim.model,sim.data;bad=[]
 for c in d.contact:
  a,b=int(m.geom_bodyid[c.geom1]),int(m.geom_bodyid[c.geom2]);ra,rb=a in sim.robot_bodies,b in sim.robot_bodies
  if ra==rb:continue
  gid=int(c.geom2 if ra else c.geom1);body=a if ra else b
  if 'floor' in m.geom(gid).name.lower() and any(x in m.body(body).name for x in ['base','wheel']):continue
  if c.dist<-.001:bad.append({'robot_body':m.body(body).name,'environment_geom':m.geom(gid).name,'penetration_m':float(-c.dist)})
 return bad

def reach_bound(sim):
 # RBY1 特定的连续躯干解析前提：两根 .35m link 水平分量抵消。
 import mujoco
 from ..simulation import body_pose
 m,d=sim.model,sim.data
 for name in ['link_torso_2','link_torso_3']:
  if not np.allclose(m.body('robot_0/'+name).pos,[0,0,.35],atol=1e-7):raise ValueError('torso reach-bound assumptions changed')
 for i in [1,2,3]:
  if not np.allclose(m.joint('robot_0/torso_'+str(i)).pos,[0,0,0]) or not np.allclose(m.body('robot_0/link_torso_'+str(i)).quat,[1,0,0,0]):raise ValueError('nonzero torso joint origin / fixed rotation')
  if not np.allclose(m.joint('robot_0/torso_'+str(i)).axis,[0,1,0]):raise ValueError('torso axes changed')
 # 检查其他固定偏置无水平移动，并验证五个 h 的肩部水平半径。
 reference={side:np.linalg.norm((np.linalg.inv(body_pose(d,m.body('robot_0/base').id))@body_pose(d,m.body(f'robot_0/link_{side}_arm_0').id))[:2,3]) for side in ['left','right']}
 base=sim.group('base').copy()
 for h in [0,.1845,.369,.5535,.738]:
  place(sim,base,h)
  for side in reference:
   radius=np.linalg.norm((np.linalg.inv(body_pose(d,m.body('robot_0/base').id))@body_pose(d,m.body(f'robot_0/link_{side}_arm_0').id))[:2,3])
   if abs(radius-reference[side])>1e-7:raise ValueError('shoulder horizontal cancellation failed')
 place(sim,base,0);bounds=[]
 for side in ['left','right']:
  shoulder=m.body(f'robot_0/link_{side}_arm_0').id;radius=0.
  for fid in [m.body(f'robot_0/ee_finger_{side[0]}{i}').id for i in [1,2]]:
   chain_bodies=[];b=fid
   while b!=shoulder:
    if b<=0:raise ValueError('finger chain not under shoulder')
    chain_bodies.append(b);b=int(m.body_parentid[b])
   chain=sum(np.linalg.norm(m.body_pos[b]) for b in chain_bodies)
   for b in chain_bodies+[shoulder]:
    for j in range(int(m.body_jntadr[b]),int(m.body_jntadr[b])+int(m.body_jntnum[b])):
     chain+=max(abs(m.jnt_range[j])) if m.jnt_type[j]==mujoco.mjtJoint.mjJNT_SLIDE else 2*np.linalg.norm(m.jnt_pos[j])
   for g in range(int(m.body_geomadr[fid]),int(m.body_geomadr[fid])+int(m.body_geomnum[fid])):
    if m.geom_contype[g] or m.geom_conaffinity[g]:radius=max(radius,chain+np.linalg.norm(m.geom_pos[g])+m.geom_rbound[g])
  bounds.append({'side':side,'shoulder_xy_radius_m':float(reference[side]),'finger_chain_radius_m':float(radius)})
 target_radius=max(np.linalg.norm(d.geom_xpos[g]-d.xpos[sim.target])+m.geom_rbound[g] for g in range(m.ngeom) if int(m.body_rootid[m.geom_bodyid[g]])==sim.target_root and (m.geom_contype[g] or m.geom_conaffinity[g]))
 return {'contact_radius_m':float(max(r['shoulder_xy_radius_m']+r['finger_chain_radius_m'] for r in bounds)+target_radius+.03),'arm_bounds':bounds,'target_collision_radius_m':float(target_radius),'inflation_m':.03,'coverage':'both arms, all yaw, continuous legal torso linkage under verified RBY1 cancellation; arbitrary arm configurations','not_search_failure':True}

def gaussian_scores(points,labels,queries,k=3,sigma=.12,max_distance=.30):
 from scipy.spatial import cKDTree
 if not len(points):return np.full(len(queries),np.nan),np.zeros(len(queries),int)
 tree=cKDTree(points);dist,idx=tree.query(queries,k=min(k,len(points)))
 if dist.ndim==1:dist=dist[:,None];idx=idx[:,None]
 supported=dist<=max_distance;weight=np.exp(-dist**2/(2*sigma**2))*supported;total=weight.sum(1)
 scores=np.divide((weight*np.asarray(labels)[idx]).sum(1),total,out=np.full(len(queries),np.nan),where=total>0)
 return scores,supported.sum(1)

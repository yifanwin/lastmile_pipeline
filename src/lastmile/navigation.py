"""保守二维 A* 路线 + 实际底盘伺服闭环；物理碰撞保留。"""
import heapq
import math
import numpy as np
import mujoco
from molmo_spaces.utils.mj_model_and_data_utils import geom_aabb
from .simulation import body_pose


def astar_grid(free,start,goal):
    start,goal=tuple(start),tuple(goal)
    if not free[start] or not free[goal]:raise ValueError(f'起点/终点不在保守导航自由空间: start={free[start]}, goal={free[goal]}')
    frontier=[(0.,0.,start)];cost={start:0.};parents={}
    while frontier:
        _,g,p=heapq.heappop(frontier)
        if g!=cost[p]:continue
        if p==goal:
            route=[p]
            while route[-1]!=start:route.append(parents[route[-1]])
            return np.array(route[::-1],dtype=int)
        for di,dj in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]:
            q=(p[0]+di,p[1]+dj)
            if not (0<=q[0]<free.shape[0] and 0<=q[1]<free.shape[1]) or not free[q]:continue
            if di and dj and (not free[p[0]+di,p[1]] or not free[p[0],p[1]+dj]):continue
            ng=g+math.hypot(di,dj)
            if ng>=cost.get(q,float('inf')):continue
            cost[q]=ng;parents[q]=p
            heapq.heappush(frontier,(ng+math.dist(q,goal),ng,q))
    raise ValueError('保守自由空间内无绕行路线；不删家具、不改碰撞')


def build_grid(sim,start,goal,table_center,table_size,spacing=.025,radius=.34):
    low=table_center[:2]-table_size[:2]/2-1.4;high=table_center[:2]+table_size[:2]/2+1.4
    low=np.minimum(low,np.minimum(start[:2],goal[:2])-.4);high=np.maximum(high,np.maximum(start[:2],goal[:2])+.4)
    xs=np.arange(low[0],high[0]+spacing,spacing);ys=np.arange(low[1],high[1]+spacing,spacing)
    xx,yy=np.meshgrid(xs,ys,indexing='ij');pts=np.c_[xx.ravel(),yy.ravel()]
    free=np.ones(xx.shape,dtype=bool);floors=np.zeros(len(pts),dtype=bool);obstacles=[]
    m,d=sim.model,sim.data
    for gid in range(m.ngeom):
        if m.geom_bodyid[gid] in sim.robot_bodies:continue
        name=m.geom(gid).name.lower()
        if 'floor' not in name and not (m.geom_contype[gid] or m.geom_conaffinity[gid]):continue
        if 'floor' in name and m.geom_type[gid]==mujoco.mjtGeom.mjGEOM_PLANE:
            floors[:]=True;continue
        center,size=geom_aabb(m,d,[gid]);lo=center-size/2;hi=center+size/2
        if np.any(hi[:2]<low) or np.any(lo[:2]>high):continue
        if 'floor' in name:
            # 地面覆盖由真实 floor mesh 投影构成，不使用房屋 AABB 充当地面。
            if m.geom_type[gid]==mujoco.mjtGeom.mjGEOM_MESH:
                mid=int(m.geom_dataid[gid]);va=int(m.mesh_vertadr[mid]);fa=int(m.mesh_faceadr[mid]);vs=m.mesh_vert[va:va+m.mesh_vertnum[mid]]
                rot=d.geom_xmat[gid].reshape(3,3);v=(vs@rot.T+d.geom_xpos[gid])[:,:2]
                for f in m.mesh_face[fa:fa+m.mesh_facenum[mid]]:
                    a,b,c=v[f];u=b-a;w=c-a;det=u[0]*w[1]-u[1]*w[0]
                    if abs(det)<1e-8:continue
                    z=pts-a;uu=(z[:,0]*w[1]-z[:,1]*w[0])/det;vv=(u[0]*z[:,1]-u[1]*z[:,0])/det
                    floors|=(uu>=-1e-6)&(vv>=-1e-6)&(uu+vv<=1+1e-6)
            elif m.geom_type[gid]==mujoco.mjtGeom.mjGEOM_BOX:
                floors|=((pts>=lo[:2])&(pts<=hi[:2])).all(axis=1)
            continue
        if hi[2]<.025 or lo[2]>1.85:continue
        gapx=np.maximum(np.maximum(lo[0]-xx,xx-hi[0]),0);gapy=np.maximum(np.maximum(lo[1]-yy,yy-hi[1]),0)
        local_radius=radius if lo[2]<.65 else .34
        free&=(gapx*gapx+gapy*gapy>local_radius*local_radius)
        obstacles.append({'name':m.geom(gid).name,'min':lo.tolist(),'max':hi.tolist()})
    floors=floors.reshape(xx.shape)
    # 圆盘边缘也必须有地面；一次侵蚀保证不穿墙外空域。
    from scipy.ndimage import distance_transform_edt
    free&=distance_transform_edt(floors)*spacing>radius
    cell=lambda pose:np.rint((np.asarray(pose)[:2]-low)/spacing).astype(int)
    np.savez_compressed(sim.run_dir/'grid_debug.npz',low=low,spacing=spacing,free=free,floors=floors)
    from .audit import write_json
    write_json(sim.run_dir/'grid_debug.json',{'obstacles':obstacles,'start':list(start),'goal':list(goal),'start_on_floor':bool(floors[tuple(cell(start))]),'goal_on_floor':bool(floors[tuple(cell(goal))])})
    route=astar_grid(free,cell(start),cell(goal));path=low+route*spacing
    # 避免网格量化终点；真实终点仍需到站闭环跟踪及物理验收。
    path=np.vstack([np.asarray(start)[:2],path,np.asarray(goal)[:2]])
    return {'low':low,'spacing':spacing,'radius':radius,'free':free,'floors':floors,'path':path,'obstacles':obstacles}


def navigation_collision(sim):
    m,d=sim.model,sim.data;bad=[]
    for ci,c in enumerate(d.contact):
        a,b=int(m.geom_bodyid[c.geom1]),int(m.geom_bodyid[c.geom2]);ra,rb=a in sim.robot_bodies,b in sim.robot_bodies
        if ra==rb:continue
        force=np.zeros(6);mujoco.mj_contactForce(m,d,ci,force)
        if c.efc_address<0 or force[0]<=1e-5:continue
        other=int(c.geom2 if ra else c.geom1);body=a if ra else b
        if 'floor' in m.geom(other).name.lower() and ('base' in m.body(body).name or 'wheel' in m.body(body).name):continue
        bad.append(m.body(body).name+' / '+m.geom(other).name)
    return sorted(set(bad))


def execute_navigation(sim,path,goal,trace,states,max_speed=.12,record_stride=10):
    """世界系相对 20D 底盘命令，按实测误差沿路线追踪。"""
    reference={'head':sim.group('head').copy(),'left_arm':sim.group('left_arm').copy(),'right_arm':sim.group('right_arm').copy(),'target':body_pose(sim.data,sim.target).copy()}
    from .protocol import torso_joints
    def record(phase):
        row=sim.sample('left',0.)
        bad=navigation_collision(sim)
        if bad:raise RuntimeError('navigation_collision:'+str(bad))
        if row['illegal']:raise RuntimeError('navigation_illegal:'+str(row['illegal']))
        if np.max(np.abs(sim.group('torso')-torso_joints(0)))>.002:raise RuntimeError('navigation_torso_protocol')
        for group in ['head','left_arm','right_arm']:
            if np.max(np.abs(sim.group(group)-reference[group]))>.002:raise RuntimeError('navigation_idle_drift:'+group)
        if np.linalg.norm(sim.data.xpos[sim.target]-reference['target'][:3,3])>.002:raise RuntimeError('target_displaced_by_navigation')
        row.update(phase=phase,forbidden_navigation_contact=bad);trace.append(row)
        if len(trace)%record_stride==0:states.append({'time_s':float(sim.data.time),'phase':phase,'qpos':sim.data.qpos.copy()})
    def control(target):
        actual=sim.group('base');delta=np.asarray(target)-actual;delta[2]=np.arctan2(np.sin(delta[2]),np.cos(delta[2]))
        action=np.zeros(20);action[:3]=delta;action[10]=action[18]=-.05;sim.action(action,phase='navigation')
        for _ in range(5):mujoco.mj_step(sim.model,sim.data);record(current_phase)
    # 起点静置；不是抓取成功，也不伪造伸臂失败轨迹。
    start=sim.group('base').copy()
    current_phase='start'
    for _ in range(100):control(start)
    current_phase='navigation'
    # 连续弧长参考，避免逐网格点重新加速；加减速限制不放宽物理阈值。
    lengths=np.linalg.norm(np.diff(path,axis=0),axis=1);arc=np.r_[0.,np.cumsum(lengths)];total=arc[-1]
    progress=0.;speed=0.;accel=.01
    while progress<total-1e-9:
        speed=min(max_speed,speed+accel*.02,np.sqrt(max(0.,2*accel*(total-progress))))
        progress=min(total,progress+max(speed,1e-5)*.02)
        k=min(len(lengths)-1,int(np.searchsorted(arc,progress,side='right')-1));alpha=(progress-arc[k])/max(lengths[k],1e-10)
        point=path[k]+alpha*(path[k+1]-path[k]);control(np.r_[point,start[2]])
    for _ in range(100):control(np.r_[goal[:2],start[2]])
    # 同样平滑转向，底盘始终由伺服驱动而非直接写 qpos。
    begin=sim.group('base').copy();delta=np.arctan2(np.sin(goal[2]-begin[2]),np.cos(goal[2]-begin[2]));turn=0.;angular_speed=0.;angular_accel=.02
    while turn<abs(delta)-1e-9:
        angular_speed=min(.05,angular_speed+angular_accel*.02,np.sqrt(max(0.,2*angular_accel*(abs(delta)-turn))))
        turn=min(abs(delta),turn+max(angular_speed,1e-5)*.02);control(np.r_[goal[:2],begin[2]+np.sign(delta)*turn])
    for _ in range(150):control(goal)
    actual=sim.group('base');yaw=np.arctan2(np.sin(actual[2]-goal[2]),np.cos(actual[2]-goal[2]))
    if np.linalg.norm(actual[:2]-goal[:2])>.002 or abs(yaw)>.00349:raise RuntimeError('navigation_arrival_tolerance')
    return {'actual_arrival':actual.tolist(),'goal':np.asarray(goal).tolist(),'translation_error_m':float(np.linalg.norm(actual[:2]-goal[:2])),'yaw_error_rad':float(abs(yaw)),'physical_time_s':float(sim.data.time),'forbidden_contacts':0,'target_displacement_m':float(np.linalg.norm(sim.data.xpos[sim.target]-reference['target'][:3,3]))}

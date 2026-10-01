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
    # 每个几何拐点停车，速度连续到 0 后再改方向，避免栅格折线的侧向冲击。
    # 输入路线先做保守可见性简化，不能跨越障碍角点。
    for first,last in zip(path[:-1],path[1:]):
        vector=last-first;length=float(np.linalg.norm(vector))
        if length<1e-8:continue
        progress=0.;speed=0.;accel=.005
        while progress<length-1e-9:
            speed=min(max_speed,speed+accel*.02,np.sqrt(max(0.,2*accel*(length-progress))))
            progress=min(length,progress+max(speed,1e-5)*.02)
            control(np.r_[first+vector*(progress/length),start[2]])
        for _ in range(30):control(np.r_[last,start[2]])
    for _ in range(100):control(np.r_[goal[:2],start[2]])
    # 同样平滑转向，底盘始终由伺服驱动而非直接写 qpos。
    begin=sim.group('base').copy();delta=np.arctan2(np.sin(goal[2]-begin[2]),np.cos(goal[2]-begin[2]));turn=0.;angular_speed=0.;angular_accel=.005
    while turn<abs(delta)-1e-9:
        angular_speed=min(.025,angular_speed+angular_accel*.02,np.sqrt(max(0.,2*angular_accel*(abs(delta)-turn))))
        turn=min(abs(delta),turn+max(angular_speed,1e-5)*.02);control(np.r_[goal[:2],begin[2]+np.sign(delta)*turn])
    for _ in range(150):control(goal)
    actual=sim.group('base');yaw=np.arctan2(np.sin(actual[2]-goal[2]),np.cos(actual[2]-goal[2]))
    if np.linalg.norm(actual[:2]-goal[:2])>.002 or abs(yaw)>.00349:raise RuntimeError('navigation_arrival_tolerance')
    return {'actual_arrival':actual.tolist(),'goal':np.asarray(goal).tolist(),'translation_error_m':float(np.linalg.norm(actual[:2]-goal[:2])),'yaw_error_rad':float(abs(yaw)),'physical_time_s':float(sim.data.time),'forbidden_contacts':0,'target_displacement_m':float(np.linalg.norm(sim.data.xpos[sim.target]-reference['target'][:3,3]))}


def build_configuration_grid(sim,start,goal,table_center,table_size,spacing=.025,margin=.025):
    """固定导航姿态的几何 Minkowski 过滤；不以 .34m 圆盘代表整台机器人。

    各机器人身体的碰撞 geom 合成保守 world AABB，并只与相同高度的
    环境碰撞 geom 配对。包含双臂/头/手指，目标也保留为障碍。最终转向单独检查。
    """
    from .workflow.station_geometry import floor_mask
    from scipy.ndimage import distance_transform_edt
    m,d=sim.model,sim.data;start=np.asarray(start);goal=np.asarray(goal)
    if np.linalg.norm(sim.group('base')-start)>1e-6:raise ValueError('configuration grid requires actual start pose')
    low=np.minimum(table_center[:2]-table_size[:2]/2-1.4,np.minimum(start[:2],goal[:2])-.6)
    high=np.maximum(table_center[:2]+table_size[:2]/2+1.4,np.maximum(start[:2],goal[:2])+.6)
    xs=np.arange(low[0],high[0]+spacing,spacing);ys=np.arange(low[1],high[1]+spacing,spacing)
    xx,yy=np.meshgrid(xs,ys,indexing='ij');free=np.ones(xx.shape,bool);parts={};obstacles=[]
    for gid in range(m.ngeom):
        if not (m.geom_contype[gid] or m.geom_conaffinity[gid]):continue
        center,size=geom_aabb(m,d,[gid]);lo=center-size/2;hi=center+size/2;bid=int(m.geom_bodyid[gid])
        if bid in sim.robot_bodies:
            if bid not in parts:parts[bid]=[lo.copy(),hi.copy()]
            else:parts[bid]=[np.minimum(parts[bid][0],lo),np.maximum(parts[bid][1],hi)]
        elif 'floor' not in m.geom(gid).name.lower() and m.geom_type[gid]!=mujoco.mjtGeom.mjGEOM_PLANE:
            if np.any(hi[:2]<low-.8) or np.any(lo[:2]>high+.8):continue
            obstacles.append({'geom_id':gid,'name':m.geom(gid).name,'min':lo.tolist(),'max':hi.tolist()})
    footprint=[]
    for bid,(lo,hi) in parts.items():
        lo=lo.copy();hi=hi.copy();lo[:2]-=start[:2];hi[:2]-=start[:2]
        footprint.append({'body':m.body(bid).name,'min':lo.tolist(),'max':hi.tolist()})
        for obstacle in obstacles:
            a=np.array(obstacle['min']);b=np.array(obstacle['max'])
            if hi[2]+margin<a[2] or lo[2]-margin>b[2]:continue
            amin=a[:2]-hi[:2]-margin;bmax=b[:2]-lo[:2]+margin
            free&=~((xx>=amin[0])&(xx<=bmax[0])&(yy>=amin[1])&(yy<=bmax[1]))
    floor_radius=max(np.linalg.norm([x,y]) for p in footprint for x in [p['min'][0],p['max'][0]] for y in [p['min'][1],p['max'][1]])+margin
    floors=floor_mask(sim,xs,ys);clear=distance_transform_edt(np.pad(floors,1))[1:-1,1:-1]*spacing
    free&=clear>floor_radius
    cell=lambda pose:np.rint((np.asarray(pose)[:2]-low)/spacing).astype(int)
    np.savez_compressed(sim.run_dir/'grid_debug.npz',low=low,spacing=spacing,free=free,floors=floors)
    from .audit import write_json
    write_json(sim.run_dir/'grid_debug.json',{'obstacles':obstacles,'robot_footprint':footprint,'floor_radius_m':float(floor_radius),'margin_m':margin,'fixed_yaw_rad':float(start[2]),'start':start.tolist(),'goal':goal.tolist()})
    route=astar_grid(free,cell(start),cell(goal));path=np.vstack([start[:2],low+route*spacing,goal[:2]])
    path=simplify_grid_path(path,free,low,spacing)
    # 连接精确端点和量化网格同样经过安全检查，不任意跳过 blocked 单元。
    for a,b in zip(path[:-1],path[1:]):
        for xy in np.linspace(a,b,max(2,int(np.linalg.norm(b-a)/(spacing/4))+1)):
            if not free[tuple(cell(xy))]:raise ValueError('精确端点连线离开导航自由空间')
    return {'low':low,'spacing':spacing,'free':free,'floors':floors,'path':path,'obstacles':obstacles,'robot_footprint':footprint,'floor_radius_m':floor_radius,'margin_m':margin}


def simplify_grid_path(path,free,low,spacing):
    """最长保守可见线段贪心简化；四分之一格采样且禁止对角穿角。"""
    path=np.asarray(path,float);out=[path[0]];i=0
    def visible(a,b):
        n=max(2,int(np.linalg.norm(b-a)/(spacing/4))+1);previous=None
        for xy in np.linspace(a,b,n):
            cell=tuple(np.rint((xy-low)/spacing).astype(int))
            if not (0<=cell[0]<free.shape[0] and 0<=cell[1]<free.shape[1]) or not free[cell]:return False
            if previous and cell[0]!=previous[0] and cell[1]!=previous[1]:
                if not free[cell[0],previous[1]] or not free[previous[0],cell[1]]:return False
            previous=cell
        return True
    while i<len(path)-1:
        j=len(path)-1
        while j>i+1 and not visible(path[i],path[j]):j-=1
        if not visible(path[i],path[j]):raise ValueError('路线包含 blocked 连线')
        if np.linalg.norm(path[j]-out[-1])>1e-8:out.append(path[j])
        i=j
    return np.asarray(out)

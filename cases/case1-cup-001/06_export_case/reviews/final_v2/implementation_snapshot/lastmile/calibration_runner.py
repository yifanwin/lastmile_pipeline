"""固定底盘的单臂 cuRobo 校准：每个独立试验从同一完整快照恢复。"""
from pathlib import Path
import json
import gc
import mujoco
import numpy as np
import torch
import yaml
from scipy.spatial.transform import Rotation
from curobo.geom.types import Cuboid,WorldConfig
from molmo_spaces.planner.curobo_planner import CuroboPlanner,CuroboPlannerConfig
from molmo_spaces.utils.mj_model_and_data_utils import body_aabb
from .audit import write_json,sha256
from .protocol import arm_only_config,waypoint_action,to_base_frame,successful_plan
from .simulation import Simulation,body_pose,pose7
from .validation import validate_trace,validate_motion_prefix


def world_obstacles(sim):
    """局部环境全根体 AABB 保守近似；从不据此声明物理不可达。"""
    base=body_pose(sim.data,sim.model.body('robot_0/base').id)
    cuboids=[]
    for body in range(1,sim.model.nbody):
        if sim.model.body_parentid[body] != 0 or body in sim.robot_bodies or body==sim.target_root:
            continue
        center,size=body_aabb(sim.model,sim.data,body,visible_only=False)
        if min(size)<=1e-5:continue
        if np.linalg.norm(np.maximum(np.abs(center[:2]-base[:2,3])-size[:2]/2,0))>2.0:continue
        transform=np.eye(4);transform[:3,3]=center
        cuboids.append(Cuboid(name=sim.model.body(body).name,pose=pose7(to_base_frame(transform,base)),dims=size.tolist()))
    return WorldConfig(cuboid=cuboids)


def tick(sim,side,h,trace):
    mujoco.mj_step(sim.model,sim.data)
    trace.append(sim.sample(side,h))
    observer=getattr(sim,'tick_observer',None)
    if observer is not None:observer(sim,getattr(sim,'record_phase','pick'))


def execute_path(sim,planner,goal,side,h,grippers,trace):
    base=body_pose(sim.data,sim.model.body('robot_0/base').id)
    q=sim.group(side+'_arm')
    path,result=planner.motion_plan(q.tolist(),[pose7(to_base_frame(goal,base))])
    status=str(result.status)
    if not successful_plan(result,path):return False,status
    for point in path:
        action=waypoint_action(point,sim.group(side+'_arm'),side,h,grippers)
        sim.action(action)
        for _ in range(5):tick(sim,side,h,trace)
    # 最后一帧真实跟踪到位再开始下一阶段；有上限，防止碰撞顶住后一直等。
    for _ in range(250):
        if np.max(np.abs(sim.group(side+'_arm')-np.asarray(path[-1])))<.01:break
        for _ in range(5):tick(sim,side,h,trace)
    if np.max(np.abs(sim.group(side+'_arm')-np.asarray(path[-1])))>=.01:
        return False,'tracking_timeout'
    tcp=sim.tcp(side)
    pose_error=float(np.linalg.norm(tcp[:3,3]-goal[:3,3]))
    rotation_error=float(Rotation.from_matrix(goal[:3,:3].T @ tcp[:3,:3]).magnitude())
    if pose_error>.01 or rotation_error>.05:
        return False,f'tcp_tracking_error:{pose_error:.6f}m/{rotation_error:.6f}rad'
    return True,status


def run_one(workspace,index,side,h,seed,run_dir,max_grasps=24,sim=None,world_geometry="aabb",time_dilation=1.,approach_overshoot=0.,grasp_ids=None,restore_start=True):
    W=Path(workspace);run_dir=Path(run_dir);run_dir.mkdir(parents=True,exist_ok=True)
    P=json.loads((W/'configs/protocol_v1.json').read_text())
    np.random.seed(seed);torch.manual_seed(seed)
    if sim is None:sim=Simulation(W,index,run_dir)
    if restore_start:sim.restore()
    sim.record_phase='torso_adjust'
    handle=sim.episode['provenance']['mode']=='handle'
    sim.part_annotation=None
    if handle:
        uid=sim.episode['provenance']['asset_id'];annotation_path=W/f'configs/parts/{uid}.json'
        if not annotation_path.exists():raise ValueError('缺少把手标注；禁止回退普通抓取')
        annotation=json.loads(annotation_path.read_text())
        if annotation.get('human_confirmed') is not True or not annotation.get('allowed_regions') or not annotation.get('allowed_grasp_ids') or annotation.get('local_cup_up_axis') is None:raise ValueError('把手区域/抓法尚未确认；禁止回退普通抓取')
        sim.part_annotation=annotation;sim.handle_initial_pose=body_pose(sim.data,sim.target).copy()
    m,d=sim.model,sim.data
    # 先在真实动态中缓慢调整 h；不能瞬间设置 qpos 得到理想躯干。
    idle='right' if side=='left' else 'left'
    prefix_initial={'base':sim.group('base').tolist(),'head':sim.group('head').tolist(),'idle_arm':sim.group(idle+'_arm').tolist()}
    adjustment=[]
    for step in range(max(1,int(h/.001)+1)):
        current_h=min(h,step*.001)
        action=waypoint_action(sim.group(side+'_arm'),sim.group(side+'_arm'),side,current_h)
        sim.action(action,phase='torso_adjust')
        for _ in range(10):tick(sim,side,current_h,adjustment)
    for _ in range(250):tick(sim,side,h,adjustment)
    prefix_failure=validate_motion_prefix(adjustment,prefix_initial,P['thresholds'])
    write_json(run_dir/'torso_adjust_trace.json',{'initial':prefix_initial,'samples':adjustment})
    if prefix_failure:
        record={'source_index':index,'side':side,'h':h,'seed':seed,'status':'executed_failure','reason':prefix_failure}
        write_json(run_dir/'result.json',record);return record
    sim.record_phase='planning'
    measured_h=float(sim.group('torso')[1])
    torso_error=float(np.max(np.abs(sim.group('torso')-np.array([0,h,-2*h,h,0,0]))))
    base0=np.array(sim.episode['robot']['init_qpos']['base'])
    drift=float(np.linalg.norm(sim.group('base')[:2]-base0[:2]))
    if torso_error>P['thresholds']['torso_error_rad'] or drift>P['thresholds']['base_translation_m']:
        record={'source_index':index,'side':side,'h':h,'seed':seed,'status':'executed_failure',
                'reason':'torso_adjust_protocol','torso_error_rad':torso_error,'base_drift_m':drift}
        write_json(run_dir/'result.json',record);return record
    idle='right' if side=='left' else 'left'
    asset_id=sim.episode['provenance']['asset_id']
    asset=json.loads((W/f'registry/assets/{asset_id}.json').read_text())
    if handle and (annotation.get('asset_xml_sha256')!=asset['asset_xml_sha256'] or annotation.get('grasp_sha256')!=asset['grasp_sha256']):raise ValueError('把手标注源哈希已失效')
    with np.load(asset['grasp_path']) as z:grasps=z['transforms'].astype(float)
    # 确定性 SO(3) 投影修正 float16 grasp 量化误差，不改变 row ID。
    u,_,v=np.linalg.svd(grasps[:,:3,:3]);grasps[:,:3,:3]=u@v
    target_pose=body_pose(d,sim.target)
    world=target_pose@grasps
    distances=np.linalg.norm(world[:,:3,3]-sim.tcp(side)[:3,3],axis=1)
    candidates=np.argsort(distances,kind='stable')[:max_grasps] if grasp_ids is None else np.asarray(grasp_ids,dtype=int)
    if handle:
        ranked=np.argsort(distances,kind='stable') if grasp_ids is None else candidates
        candidates=np.array([g for g in ranked if g in annotation['allowed_grasp_ids']][:max_grasps],dtype=int)
    if candidates.ndim!=1 or not len(candidates) or len(candidates)>max_grasps or np.any(candidates<0) or np.any(candidates>=len(grasps)):
        raise ValueError('非法冻结 grasp 集合')
    # 所有内部候选共享同一校准起始快照；独立执行前必恢复。
    candidate_state=np.zeros_like(sim.initial_state)
    mujoco.mj_getState(m,d,candidate_state,sim.state_spec)
    np.savez_compressed(run_dir/'calibration_snapshot.npz',state=candidate_state)
    template=yaml.safe_load((sim.assets/'robots/rby1m/curobo_config/rby1m_holobase.yml').read_text())
    cfg=arm_only_config(template,side,h,sim.group(idle+'_arm'),sim.group('head'))
    kin=cfg['robot_cfg']['kinematics']
    kin['lock_joints'].update({f'torso_{i}':float(q) for i,q in enumerate(sim.group('torso'))})
    robotroot=sim.assets/'robots/rby1m/curobo_config'
    kin['urdf_path']=str(robotroot/'urdf/model_holobase.urdf');kin['asset_root_path']=str(robotroot/'urdf/meshes')
    kin['collision_spheres']=str(robotroot/'rby1m_holobase_spheres.yml');kin['usd_robot_root']=str(robotroot)
    config_path=run_dir/'planner.yml';config_path.write_text(yaml.safe_dump(cfg,sort_keys=False))
    planner=CuroboPlanner(CuroboPlannerConfig(curobo_robot_config_path=str(config_path),
        world_config=mesh_world_obstacles(sim) if world_geometry=='mesh' else world_obstacles(sim),collision_cache={'mesh':3,'obb':512},
        num_ik_seeds=P['budget']['num_ik_seeds'],num_trajopt_seeds=P['budget']['num_trajopt_seeds'],
        max_attempts=P['budget']['max_attempts'],collision_activation_distance=.01,time_dilation_factor=time_dilation))
    if planner.joint_names!=[f'{side}_arm_{i}' for i in range(7)]:raise ValueError('规划器放开了非法关节')
    base=body_pose(d,m.body('robot_0/base').id)
    fk=np.asarray(planner.fk_solve(sim.group(side+'_arm').tolist()))
    real=to_base_frame(sim.tcp(side),base)
    if np.linalg.norm(fk[:3]-real[:3,3])>.001:raise ValueError('cuRobo/MuJoCo TCP 坐标不一致')
    errors=[];result_record=None;last_dynamic_verdict=None
    ik_poses=[pose7(to_base_frame(world[i],base)) for i in candidates]
    ik_mask=planner.ik_solve_batch(ik_poses,seed_config=sim.group(side+'_arm').tolist(),num_seeds=64,disable_collision=False,pad_to=len(candidates))
    for gid in candidates[~ik_mask]:errors.append({'grasp_id':int(gid),'stage':'ik_prefilter','status':'budget_exhausted_no_solution'})
    candidates_to_execute=candidates[ik_mask]
    for grasp_id in candidates_to_execute:
        mujoco.mj_setState(m,d,candidate_state,sim.state_spec);mujoco.mj_forward(m,d)
        sim._handle_acquired=False;sim._arm_targets=None;sim._fixed_base=sim.group('base').copy();sim._last_phase='pick'
        trace=[]
        initial={'base':sim.group('base').tolist(),'head':sim.group('head').tolist(),
                 'idle_arm':sim.group(idle+'_arm').tolist(),'height_m':sim.initial_z,
                 'target_id':sim.target_name,'selected_fingers':[m.body(f'robot_0/ee_finger_{side[0]}{i}').id for i in (1,2)]}
        goal=world[grasp_id].copy();pre=goal.copy();pre[:3,3]-=goal[:3,2]*.08
        goal[:3,3]+=goal[:3,2]*approach_overshoot
        sim.record_phase='pregrasp'
        ok,status=execute_path(sim,planner,pre,side,h,(-.05,-.05),trace)
        if not ok:
            write_json(run_dir/f'grasp_{int(grasp_id):04d}_pregrasp_failed_trace.json',{'initial':initial,'samples':trace,'failure':status})
            errors.append({'grasp_id':int(grasp_id),'stage':'pregrasp','status':status});continue
        sim.record_phase='approach'
        ok,status=execute_path(sim,planner,goal,side,h,(-.05,-.05),trace)
        if not ok:
            write_json(run_dir/f'grasp_{int(grasp_id):04d}_approach_failed_trace.json',{'initial':initial,'samples':trace,'failure':status})
            errors.append({'grasp_id':int(grasp_id),'stage':'approach','status':status});continue
        sim.record_phase='close'
        grip=(0.,-.05) if side=='left' else (-.05,0.)
        a=waypoint_action(sim.group(side+'_arm'),sim.group(side+'_arm'),side,h,grip);sim.action(a)
        for _ in range(250):tick(sim,side,h,trace)
        lift=sim.tcp(side).copy();lift[2,3]+=.10
        sim.record_phase='lift'
        ok,status=execute_path(sim,planner,lift,side,h,grip,trace)
        if not ok:
            write_json(run_dir/f'grasp_{int(grasp_id):04d}_lift_failed_trace.json',{'initial':initial,'samples':trace,'failure':status})
            errors.append({'grasp_id':int(grasp_id),'stage':'lift','status':status});continue
        sim.record_phase='hold'
        for _ in range(550):tick(sim,side,h,trace)
        verdict=validate_trace(trace,initial,P['thresholds'],handle=handle)
        record={'source_index':index,'side':side,'h':h,'seed':seed,'grasp_id':int(grasp_id),**verdict}
        write_json(run_dir/f'grasp_{int(grasp_id):04d}_trace.json',{'initial':initial,'samples':trace})
        if verdict['status']=='verified_success':result_record=record;break
        last_dynamic_verdict=record.copy()
        errors.append({'grasp_id':int(grasp_id),'stage':'strict_pick',**verdict})
    if result_record is None and last_dynamic_verdict is not None:
        result_record=last_dynamic_verdict
    if result_record is None:
        result_record={'source_index':index,'side':side,'h':h,'seed':seed,
                       'status':'budget_exhausted_no_solution','reason':'see_candidate_attempts'}
    result_record.update(protocol_sha256=sha256(W/'configs/protocol_v1.json'),
                         benchmark_sha256=sim.episode['provenance']['benchmark_sha256'],
                         attempts=errors,planner_world='actual collision geom mesh' if world_geometry=='mesh' else 'local root-body AABB conservative approximation',
                         grasp_row_ids=candidates.tolist(),time_dilation_factor=time_dilation,approach_overshoot_m=approach_overshoot,adjustment_samples=len(adjustment))
    write_json(run_dir/'result.json',result_record)
    del planner;gc.collect();torch.cuda.empty_cache()
    return result_record


def mesh_world_obstacles(sim,max_distance=2.):
    """按真实 MuJoCo 碰撞 geom 合并世界 mesh，保留桌腿间隙和凹形边界。"""
    from curobo.geom.types import Mesh
    import trimesh
    from molmo_spaces.utils.mj_model_and_data_utils import geom_aabb
    m,d=sim.model,sim.data
    inv_base=np.linalg.inv(body_pose(d,m.body('robot_0/base').id))
    base=body_pose(d,m.body('robot_0/base').id)
    vertices,faces=[],[];offset=0;names=[]
    for gid in range(m.ngeom):
        body=int(m.geom_bodyid[gid])
        if body in sim.robot_bodies or int(m.body_rootid[body])==sim.target_root:continue
        if m.geom_contype[gid]==0 and m.geom_conaffinity[gid]==0:continue
        name=m.geom(gid).name
        # 规划中地面不作为底盘禁止接触；仿真中不关闭任何地面碰撞。
        if 'floor' in name.lower() or m.geom_type[gid]==mujoco.mjtGeom.mjGEOM_PLANE:continue
        center,size=geom_aabb(m,d,[gid])
        if np.linalg.norm(np.maximum(np.abs(center[:2]-base[:2,3])-size[:2]/2,0))>max_distance:continue
        kind=int(m.geom_type[gid]);dims=m.geom_size[gid]
        if kind==mujoco.mjtGeom.mjGEOM_MESH:
            mid=int(m.geom_dataid[gid]);v0=int(m.mesh_vertadr[mid]);f0=int(m.mesh_faceadr[mid])
            vs=m.mesh_vert[v0:v0+int(m.mesh_vertnum[mid])].copy()
            fs=m.mesh_face[f0:f0+int(m.mesh_facenum[mid])].copy()
        elif kind==mujoco.mjtGeom.mjGEOM_BOX:
            mesh=trimesh.creation.box(extents=2*dims);vs,fs=mesh.vertices,mesh.faces
        elif kind==mujoco.mjtGeom.mjGEOM_SPHERE:
            mesh=trimesh.creation.icosphere(subdivisions=2,radius=dims[0]);vs,fs=mesh.vertices,mesh.faces
        elif kind==mujoco.mjtGeom.mjGEOM_ELLIPSOID:
            mesh=trimesh.creation.icosphere(subdivisions=2);vs,fs=mesh.vertices*dims,mesh.faces
        elif kind==mujoco.mjtGeom.mjGEOM_CYLINDER:
            mesh=trimesh.creation.cylinder(radius=dims[0],height=2*dims[1],sections=32);vs,fs=mesh.vertices,mesh.faces
        elif kind==mujoco.mjtGeom.mjGEOM_CAPSULE:
            mesh=trimesh.creation.capsule(radius=dims[0],height=2*dims[1]);vs,fs=mesh.vertices,mesh.faces
        else:raise ValueError(f'不支持的实际碰撞 geom: {name} type={kind}')
        pose=np.eye(4);pose[:3,:3]=d.geom_xmat[gid].reshape(3,3);pose[:3,3]=d.geom_xpos[gid]
        local=(np.c_[vs,np.ones(len(vs))] @ (inv_base@pose).T)[:,:3]
        vertices.append(local);faces.append(fs+offset);offset+=len(vs);names.append(name)
    if not vertices:raise ValueError('没有恢复任何环境碰撞几何')
    mesh=Mesh(name='actual_collision_geoms',pose=[0,0,0,1,0,0,0],vertices=np.concatenate(vertices).tolist(),faces=np.concatenate(faces).tolist())
    write_json(sim.run_dir/'mesh_world_geometry.json',{'geom_names':names,'vertex_count':offset,'face_count':sum(len(f) for f in faces),'frame':'measured robot base frame','floor_contacts':'kept in physics; excluded from planning base obstacle model','primitive_surface_note':'box/mesh exact; sphere/ellipsoid/cylinder/capsule tessellated'})
    return WorldConfig(mesh=[mesh])

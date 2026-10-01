"""独立的只读场景恢复与 RBY1 20 维执行桥。"""
from pathlib import Path
import json
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from molmo_spaces.robots.rby1 import RBY1
from scipy.spatial.transform import Rotation
from .protocol import unpack_action, torso_joints
from .audit import write_json
from .parts import classify_contact,upright_angle

def resolve_assets(workspace):
    for parent in [Path(workspace).absolute(),*Path(workspace).absolute().parents]:
        candidate=parent/'molmospaces_data/assets'
        if candidate.exists():return candidate
    raise FileNotFoundError('找不到只读资产目录 molmospaces_data/assets')



def body_pose(data, body_id):
    p=np.eye(4);p[:3,3]=data.xpos[body_id];p[:3,:3]=data.xmat[body_id].reshape(3,3)
    return p


def pose7(matrix):
    return np.r_[matrix[:3,3], Rotation.from_matrix(matrix[:3,:3]).as_quat(scalar_first=True)].tolist()


class Simulation:
    def __init__(self, workspace, index, run_dir):
        self.workspace=Path(workspace); self.assets=resolve_assets(self.workspace)
        self.run_dir=Path(run_dir);self.run_dir.mkdir(parents=True,exist_ok=True)
        self.episode=json.loads((self.workspace/f'registry/episodes/{index:04d}.json').read_text())
        e=self.episode;self.target_name=e['task']['pickup_obj_name']
        print('loading scene',index,flush=True)
        scene_path=Path(e['provenance']['scene_xml'])
        names={b.get('name') for b in ET.parse(scene_path).getroot().iter('body')}
        missing=sorted(set(e['scene_modifications']['object_poses'])-names-set(e['scene_modifications']['added_objects']))
        if missing:
            write_json(self.run_dir/'restoration_failure.json',{'source_index':index,'status':'infrastructure_error','reason':'source_scene_missing_restore_bodies','missing_bodies':missing})
            raise ValueError('源场景缺恢复对象：'+', '.join(missing))
        spec=mujoco.MjSpec.from_file(str(scene_path))
        robot=mujoco.MjSpec.from_file(str(self.assets/'robots/rby1m/rby1_v1.2_site_control.xml'))
        RBY1._set_gripper_position_servos(robot)
        for body in robot.bodies:
            if body.name.startswith('robot_0/'):
                body.gravcomp=1.
        spec.attach(robot,prefix='',frame=spec.worldbody.add_frame())
        for name,relpath in e['scene_modifications']['added_objects'].items():
            obj=mujoco.MjSpec.from_file(str(self.assets/relpath))
            root=obj.worldbody.bodies[0]
            root.name=name.split('/')[-1]
            if root.first_joint() is None:
                root.add_freejoint(name='free')
            spec.worldbody.add_frame().attach_body(root,'/'.join(name.split('/')[:-1])+'/', '')
        if e['scene_modifications'].get('removed_objects'):raise ValueError('当前恢复器不支持 removed_objects；禁止忽略源修改')
        missing=sorted(set(e['scene_modifications']['object_poses'])-{b.name for b in spec.bodies})
        if missing:
            write_json(self.run_dir/'restoration_failure.json',{'source_index':index,'status':'infrastructure_error','reason':'source_scene_missing_restore_bodies','missing_bodies':missing})
            raise ValueError('当前场景与 benchmark 不匹配，缺恢复对象：'+', '.join(missing))
        spec.option.timestep=.004
        print('compiling scene',index,flush=True)
        self.model=spec.compile();self.data=mujoco.MjData(self.model)
        mujoco.mj_saveModel(self.model,str(self.run_dir/'compiled_model.mjb'),None)
        m,d=self.model,self.data
        print('compiled',m.nq,m.ngeom,flush=True)
        self.robot_bodies={i for i in range(m.nbody) if m.body(i).name.startswith('robot_0/')}
        self.target=m.body(self.target_name).id
        self.target_root=int(m.body_rootid[self.target])
        for name,pose in e['scene_modifications']['object_poses'].items():
            b=m.body(name)
            joint=int(m.body_jntadr[b.id])
            if joint < 0 or m.jnt_type[joint] != mujoco.mjtJoint.mjJNT_FREE:
                raise ValueError('恢复姿态对象非自由关节：'+name)
            adr=int(m.jnt_qposadr[joint]);d.qpos[adr:adr+7]=pose
        for group,values in e['robot']['init_qpos'].items():
            if group=='base': names=['base_x','base_y','base_theta']
            elif group.endswith('gripper'):
                side=group[0];names=[f'gripper_finger_{side}1',f'gripper_finger_{side}2']
            else:names=[f'{group}_{i}' for i in range(len(values))]
            for name,value in zip(names,values):
                d.qpos[m.joint('robot_0/'+name).qposadr[0]]=value
        self.qpos_adr={m.joint(i).name.removeprefix('robot_0/'):int(m.jnt_qposadr[i]) for i in range(m.njnt) if m.joint(i).name.startswith('robot_0/')}
        self.act={m.actuator(i).name.removeprefix('robot_0/'):i for i in range(m.nu) if m.actuator(i).name.startswith('robot_0/')}
        mujoco.mj_forward(m,d)
        self.hold_ctrl()
        self.initial_z=float(d.xpos[self.target][2])
        self.local_up=d.xmat[self.target].reshape(3,3).T @ np.array([0.,0.,1.])
        if np.linalg.norm(d.xpos[self.target]-np.array(e['task']['pickup_obj_start_pose'][:3]))>1e-5:
            raise ValueError('目标恢复位置错配')
        self.state_spec=mujoco.mjtState.mjSTATE_INTEGRATION
        self.initial_state=np.zeros(mujoco.mj_stateSize(m,self.state_spec))
        mujoco.mj_getState(m,d,self.initial_state,self.state_spec)
        np.savez_compressed(self.run_dir/'initial_snapshot.npz',state=self.initial_state)
        self.initial_qpos=d.qpos.copy()
        write_json(self.run_dir/'restoration.json', {'source_index':index,'target':self.target_name,
            'restored_target_xyz':d.xpos[self.target].tolist(),'source_target_xyz':e['task']['pickup_obj_start_pose'][:3],
            'object_pose_count':len(e['scene_modifications']['object_poses']),
            'added_object_count':len(e['scene_modifications']['added_objects']),
            'qpos_count':m.nq,'geom_count':m.ngeom,'status':'compiled_and_restored_not_grasp_success'})

    def q(self,names):
        return np.array([self.data.qpos[self.qpos_adr[n]] for n in names])

    def group(self,name):
        if name=='base':return self.q(['base_x','base_y','base_theta'])
        return self.q([f'{name}_{i}' for i in range(6 if name=='torso' else 2 if name=='head' else 7)])

    def hold_ctrl(self):
        for side in ('left','right'):
            for i,q in enumerate(self.group(side+'_arm')):
                self.data.ctrl[self.act[f'{side}_arm_{i+1}_act']]=q
            q=self.q([f'gripper_finger_{side[0]}1'])[0]
            self.data.ctrl[self.act[side+'_finger_act']]=q
        for i,q in enumerate(self.group('torso')):
            self.data.ctrl[self.act[f'link{i+1}_act']]=q
        for i,q in enumerate(self.group('head')):
            self.data.ctrl[self.act[f'head_{i}_act']]=q
        for name,q in zip(('base_x','base_y','base_theta'), self.group('base')):
            self.data.ctrl[self.act[name+'_act']]=q

    def restore(self):
        self._handle_acquired=False
        self._fixed_base=None
        self._last_phase=None
        self._arm_targets=None
        mujoco.mj_setState(self.model,self.data,self.initial_state,self.state_spec)
        mujoco.mj_forward(self.model,self.data)

    def action(self,action,phase='pick'):
        a=unpack_action(action,phase=phase)
        if getattr(self,'_arm_targets',None) is None:
            self._arm_targets={s:self.group(s+'_arm').copy() for s in ('left','right')}
        if phase!='navigation' and (getattr(self,'_fixed_base',None) is None or getattr(self,'_last_phase',None)=='navigation'):
            self._fixed_base=self.group('base').copy()
        self._last_phase=phase
        for side in ('left','right'):
            if np.any(a[side+'_arm']!=0):self._arm_targets[side]=self.group(side+'_arm')+a[side+'_arm']
            for i,q in enumerate(self._arm_targets[side]):
                self.data.ctrl[self.act[f'{side}_arm_{i+1}_act']]=q
            self.data.ctrl[self.act[side+'_finger_act']]=a[side+'_gripper'][0]
        for i,q in enumerate(torso_joints(a['torso'][0])):
            self.data.ctrl[self.act[f'link{i+1}_act']]=q
        base_target=self.group('base')+a['base'] if phase=='navigation' else self._fixed_base
        for name,q in zip(('base_x','base_y','base_theta'),base_target):
            self.data.ctrl[self.act[name+'_act']]=q

    def tcp(self,side):
        site=self.model.site('robot_0/ee_site_'+side[0]).id
        p=np.eye(4);p[:3,3]=self.data.site_xpos[site];p[:3,:3]=self.data.site_xmat[site].reshape(3,3)
        return p

    def sample(self,side,h):
        m,d=self.model,self.data
        annotation=getattr(self,'part_annotation',None)
        handle_fingers=set();forbidden_part=False
        fingers=[m.body(f'robot_0/ee_finger_{side[0]}{i}').id for i in (1,2)]
        touched=set();support=False;illegal=[]
        forces={}
        for ci,c in enumerate(d.contact):
            # MuJoCo 的软接触可在 dist > 0 时承力；不得漏掉这类支撑。
            force=np.zeros(6);mujoco.mj_contactForce(m,d,ci,force)
            bearing=bool(c.efc_address>=0 and force[0]>1e-6)
            if c.dist>0 and not bearing:continue
            a,b=int(m.geom_bodyid[c.geom1]),int(m.geom_bodyid[c.geom2])
            ta,tb=int(m.body_rootid[a])==self.target_root,int(m.body_rootid[b])==self.target_root
            ra,rb=a in self.robot_bodies,b in self.robot_bodies
            if ta != tb:
                other=b if ta else a
                if other in fingers and bearing:
                    touched.add(other);forces[other]=forces.get(other,0.)+float(force[0])
                    if annotation:
                        part=classify_contact(c.pos,body_pose(d,self.target),annotation['allowed_regions'],annotation['forbidden_regions'],annotation['contact_tolerance_m'])
                        if part=='handle':handle_fingers.add(other)
                        else:forbidden_part=True
                elif other in self.robot_bodies:
                    if c.dist < -.001:illegal.append('nonfinger_target:'+m.body(other).name)
                elif other not in fingers and bearing:support=True
            elif ra and rb and a!=b and c.dist < -.002:
                illegal.append('self_collision:'+m.body(a).name+':'+m.body(b).name)
            elif ra != rb and not (ta or tb) and c.dist < -.001:
                other_geom=int(c.geom2 if ra else c.geom1)
                floor='floor' in m.geom(other_geom).name.lower()
                robot_body=a if ra else b
                base_contact=('base' in m.body(robot_body).name or 'wheel' in m.body(robot_body).name)
                if not (floor and base_contact):illegal.append('environment:'+m.geom(other_geom).name)
        target_pose=body_pose(d,self.target)
        idle='right' if side=='left' else 'left'
        semantic={}
        if annotation:
            acquired_before=getattr(self,'_handle_acquired',False)
            if handle_fingers==set(fingers) and not forbidden_part:self._handle_acquired=True
            ref=self.handle_initial_pose
            motion=(np.linalg.norm(target_pose[:3,3]-ref[:3,3])>annotation['max_pregrasp_translation_m'] or Rotation.from_matrix(ref[:3,:3].T@target_pose[:3,:3]).magnitude()>annotation['max_pregrasp_rotation_rad'])
            semantic={'handle_fingers':sorted(handle_fingers),'forbidden_part_contact':forbidden_part,'pregrasp_motion_violation':bool(motion and not acquired_before)}
        return {**semantic,'time_s':float(d.time),'target_id':self.target_name,'base':self.group('base').tolist(),
                'torso':self.group('torso').tolist(),'head':self.group('head').tolist(),
                'idle_arm':self.group(idle+'_arm').tolist(),'h':h,'illegal':sorted(set(illegal)),
                'selected_arm_q':self.group(side+'_arm').tolist(),
                'selected_gripper_q':self.q([f'gripper_finger_{side[0]}1',f'gripper_finger_{side[0]}2']).tolist(),
                'ctrl':d.ctrl.tolist(), 'tcp_pose':self.tcp(side).tolist(), 'target_pose':target_pose.tolist(),
                'height_m':float(d.xpos[self.target][2]),'support_contact':support,
                'selected_fingers':sorted(touched),'finger_normal_forces_N':forces,'contact_sampler_version':'force-aware-v2','relative_pose':np.linalg.solve(self.tcp(side),target_pose).tolist(),
                'upright_rad':upright_angle(target_pose,annotation['local_cup_up_axis']) if annotation else float(np.arccos(np.clip((target_pose[:3,:3] @ self.local_up)[2],-1.,1.)))}


    @classmethod
    def from_cached(cls, workspace, index, cache_dir, run_dir):
        """使用本工作区已经完整编译的模型与快照，不重新读取数千个 NAS mesh。"""
        sim=cls.__new__(cls)
        sim.workspace=Path(workspace).absolute();sim.assets=resolve_assets(sim.workspace)
        sim.run_dir=Path(run_dir).absolute();sim.run_dir.mkdir(parents=True,exist_ok=True)
        cache_dir=Path(cache_dir).absolute()
        metadata=json.loads((cache_dir/'restoration.json').read_text())
        if metadata['source_index']!=index:raise ValueError('缓存模型来源错配')
        sim.episode=json.loads((sim.workspace/f'registry/episodes/{index:04d}.json').read_text())
        sim.target_name=sim.episode['task']['pickup_obj_name']
        sim.model=m=mujoco.MjModel.from_binary_path(str(cache_dir/'compiled_model.mjb'))
        sim.data=d=mujoco.MjData(m)
        for side in ('left','right'):
            act=m.actuator('robot_0/'+side+'_finger_act').id
            if not (m.actuator_biasprm[act,1]<0 and np.allclose(m.actuator_ctrlrange[act],[-.05,0.])):
                raise ValueError('缓存模型未应用位置夹爪伺服')
        sim.state_spec=mujoco.mjtState.mjSTATE_INTEGRATION
        sim.initial_state=np.load(cache_dir/'initial_snapshot.npz')['state']
        sim.restore()
        sim.robot_bodies={i for i in range(m.nbody) if m.body(i).name.startswith('robot_0/')}
        sim.target=m.body(sim.target_name).id;sim.target_root=int(m.body_rootid[sim.target])
        sim.qpos_adr={m.joint(i).name.removeprefix('robot_0/'):int(m.jnt_qposadr[i]) for i in range(m.njnt) if m.joint(i).name.startswith('robot_0/')}
        sim.act={m.actuator(i).name.removeprefix('robot_0/'):i for i in range(m.nu) if m.actuator(i).name.startswith('robot_0/')}
        sim.initial_z=float(d.xpos[sim.target][2]);sim.initial_qpos=d.qpos.copy()
        sim.local_up=d.xmat[sim.target].reshape(3,3).T @ np.array([0.,0.,1.])
        np.savez_compressed(sim.run_dir/'initial_snapshot.npz',state=sim.initial_state)
        link=sim.run_dir/'compiled_model.mjb'
        if not link.exists():link.symlink_to(cache_dir/'compiled_model.mjb')
        write_json(sim.run_dir/'restoration.json',{**metadata,'cache_source':str(cache_dir)})
        return sim

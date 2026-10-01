#!/usr/bin/env python3
"""CPU: 编译真实 MJCF，统一限位，并比较 URDF 与 MJCF 双 TCP 正运动学。"""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
from lastmile.audit import write_json, sha256
from lastmile.protocol import torso_joints, arm_only_config

W = Path(__file__).absolute().parents[1]
A = W.parent/'molmospaces_data/assets'
P = json.loads((W/'configs/protocol_v1.json').read_text())
robot_xml = A/'robots/rby1m/rby1_v1.2_site_control.xml'
urdf = A/'robots/rby1m/curobo_config/urdf/model_holobase.urdf'
model = mujoco.MjSpec.from_file(str(robot_xml)).compile()
data = mujoco.MjData(model)
u = ET.parse(urdf).getroot()
joints = {j.get('name'):j for j in u.findall('joint')}


def urdf_fk(values):
    poses = {'world':np.eye(4)}
    remaining = list(joints.values())
    while remaining:
        ready = [j for j in remaining if j.find('parent').get('link') in poses]
        if not ready:
            raise ValueError('URDF 关节图不连通')
        for j in ready:
            origin = j.find('origin')
            transform = np.eye(4)
            if origin is not None:
                transform[:3,3] = np.fromstring(origin.get('xyz','0 0 0'), sep=' ')
                transform[:3,:3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy','0 0 0'), sep=' ')).as_matrix()
            q = values.get(j.get('name'), 0.)
            axis = j.find('axis')
            if j.get('type') != 'fixed':
                v = np.fromstring(axis.get('xyz','1 0 0') if axis is not None else '1 0 0', sep=' ')
                motion = np.eye(4)
                if j.get('type') == 'prismatic':
                    motion[:3,3] = v*q
                else:
                    motion[:3,:3] = Rotation.from_rotvec(v*q).as_matrix()
                transform = transform @ motion
            poses[j.find('child').get('link')] = poses[j.find('parent').get('link')] @ transform
            remaining.remove(j)
    return poses

limit_rows = []
for i in range(6):
    name = f'torso_{i}'
    mj_limit = model.joint('robot_0/'+name).range
    ur_limit = joints[name].find('limit')
    limit_rows.append({'joint':name,'mjcf':mj_limit.tolist(),
                       'urdf':[float(ur_limit.get('lower')),float(ur_limit.get('upper'))]})
    if not np.allclose(mj_limit, limit_rows[-1]['urdf']):
        raise ValueError('MJCF/URDF 躯干限位不一致')
rows = []
for h in P['h_candidates']:
    torso = torso_joints(h)
    for i, q in enumerate(torso):
        if not model.joint(f'robot_0/torso_{i}').range[0] <= q <= model.joint(f'robot_0/torso_{i}').range[1]:
            raise ValueError('协议 h 超过模型限位')
    # 两个正例初始臂姿态相同；不以初始臂姿态代表完整工作空间。
    values = {f'torso_{i}':q for i,q in enumerate(torso)}
    for side in ('left','right'):
        values.update({f'{side}_arm_{i}':q for i,q in enumerate([.5,0,0,-2.3,0,-.5,0])})
    for name, q in values.items():
        data.qpos[model.joint('robot_0/'+name).qposadr[0]] = q
    mujoco.mj_forward(model, data)
    fk = urdf_fk(values)
    for side in ('left','right'):
        site = model.site('robot_0/ee_site_'+side[0]).id
        tcp = fk['ee_'+side+'_tcp']
        delta = data.site_xpos[site] - tcp[:3,3]
        angle = Rotation.from_matrix(tcp[:3,:3].T @ data.site_xmat[site].reshape(3,3)).magnitude()
        rows.append({'h':h,'side':side,'shoulder_xyz':data.xpos[model.body('robot_0/link_'+side+'_arm_0').id].tolist(),
                     'tcp_xyz':data.site_xpos[site].tolist(),'urdf_tcp_xyz':tcp[:3,3].tolist(),
                     'position_delta_m':delta.tolist(),'base_frame_position_error_m':float(np.linalg.norm(data.site_xpos[site]-data.xpos[model.body('robot_0/base').id]-tcp[:3,3])), 'rotation_error_rad':float(angle)})
# 配置生成不是动态验收；保留全身碰撞链且唯一优化变量为指定手臂。
template = yaml.safe_load((A/'robots/rby1m/curobo_config/rby1m_holobase.yml').read_text())
for side in ('left','right'):
    for h in P['h_candidates']:
        cfg = arm_only_config(template, side, h, [.5,0,0,-2.3,0,-.5,0], [0,0])
        kin = cfg['robot_cfg']['kinematics']
        kin['urdf_path'] = str(urdf)
        kin['asset_root_path'] = str(urdf.parent/'meshes')
        kin['collision_spheres'] = str(A/'robots/rby1m/curobo_config/rby1m_holobase_spheres.yml')
        kin['usd_robot_root'] = str(urdf.parent.parent)
        out = W/f'registry/planner_configs/{side}_h{h:.4f}.yml'
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(yaml.safe_dump(cfg, sort_keys=False))
write_json(W/'reports/model_calibration.json', {
    'robot_xml_sha256':sha256(robot_xml),'urdf_sha256':sha256(urdf),
    'mujoco_version':mujoco.__version__,'joint_count':model.njnt,
    'torso_limits':limit_rows,'fk_samples':rows,
    'max_fk_translation_error_m':max(float(np.linalg.norm(r['position_delta_m'])) for r in rows),
    'max_fk_rotation_error_rad':max(r['rotation_error_rad'] for r in rows),
    'max_base_frame_fk_translation_error_m':max(r['base_frame_position_error_m'] for r in rows),
    'frame_note':'MJCF 外层根的 5mm 抬升必须在真实 base pose 中去除；不要使用 z=0 的理想底盘系',
    'status':'kinematics_measured_not_dynamic_calibration'})
print('真实 MJCF 编译完成，躯干限位匹配；FK 对照已保存。')
print('最大 FK 平移差',max(np.linalg.norm(r['position_delta_m']) for r in rows))

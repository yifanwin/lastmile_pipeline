"""部件接触几何工具；仅用经确认的物体局部区域，不能靠 TCP/geom 名称代替。"""
import numpy as np
from .protocol import finite_vector


def classify_contact(point_world, object_pose, allowed_regions, forbidden_regions, tolerance_m):
    point=finite_vector(point_world,3)
    pose=np.asarray(object_pose,dtype=float)
    if pose.shape!=(4,4) or not np.isfinite(pose).all():raise ValueError('物体位姿无效')
    local=np.linalg.solve(pose,np.r_[point,1.])[:3]
    def contains(region,tolerance):
        low=finite_vector(region['min'],3);high=finite_vector(region['max'],3)
        if np.any(low>=high):raise ValueError('部件区域无效')
        return bool(np.all(local>=low-tolerance) and np.all(local<=high+tolerance))
    # 禁区优先；重叠/歧义区域不收入把手。
    if any(contains(r,tolerance_m) for r in forbidden_regions):return 'forbidden'
    if any(contains(r,0.) for r in allowed_regions):return 'handle'
    return 'unknown'


def upright_angle(object_pose, local_up_axis):
    axis=finite_vector(local_up_axis,3)
    if not np.isclose(np.linalg.norm(axis),1.):raise ValueError('杯口轴必须为单位向量')
    pose=np.asarray(object_pose,dtype=float)
    return float(np.arccos(np.clip((pose[:3,:3]@axis)[2],-1.,1.)))

"""20 维动作、单臂规划配置与合规检查；不放开额外自由度。"""
from copy import deepcopy
import numpy as np

ORDER = (("base", 3), ("left_arm", 7), ("left_gripper", 1),
         ("right_arm", 7), ("right_gripper", 1), ("torso", 1))


def finite_vector(value, size):
    a = np.asarray(value, dtype=float)
    if a.shape != (size,) or not np.isfinite(a).all():
        raise ValueError(f"需要有限的 {size} 维向量，实际 {a.shape}")
    return a


def torso_joints(h, limits=(0., .738)):
    h = float(h)
    if not np.isfinite(h) or not limits[0] <= h <= limits[1]:
        raise ValueError("h 超出协议范围；不得静默裁剪")
    return np.array([0., h, -2*h, h, 0., 0.])


def unpack_action(action, *, phase="navigation"):
    if phase not in ("navigation", "pick", "torso_adjust"):
        raise ValueError("未知执行阶段")
    a = finite_vector(action, 20)
    result, offset = {}, 0
    for name, size in ORDER:
        result[name] = a[offset:offset+size].copy()
        offset += size
    torso_joints(result["torso"][0])
    for side in ("left", "right"):
        if not -.05 <= result[f"{side}_gripper"][0] <= 0:
            raise ValueError("RBY1 夹爪绝对控制范围为 [-.05, 0]")
    if phase != "navigation" and np.any(result["base"] != 0):
        raise ValueError("操作阶段底盘增量必须为零")
    return result


def waypoint_action(arm_q, current_q, side, h, grippers=(-.05, -.05)):
    """cuRobo 绝对关节位置 → 相对动作；未选臂增量为零。"""
    if side not in ("left", "right"):
        raise ValueError("未知手臂")
    a = np.zeros(20)
    start = 3 if side == "left" else 11
    a[start:start+7] = finite_vector(arm_q, 7) - finite_vector(current_q, 7)
    a[10], a[18], a[19] = *finite_vector(grippers, 2), h
    unpack_action(a, phase="pick")
    return a


def arm_only_config(template, side, h, idle_arm, head, grippers=(-.05, -.05)):
    """局部底盘系规划：底盘锁零，躯干/闲置臂/头/夹爪同步实测状态。

    保留原全身 collision_link_names；需对每次高度/闲置臂状态重新构建规划器。
    """
    if side not in ("left", "right"):
        raise ValueError("未知手臂")
    cfg = deepcopy(template)
    kin = cfg["robot_cfg"]["kinematics"]
    active = [f"{side}_arm_{i}" for i in range(7)]
    idle = "right" if side == "left" else "left"
    locks = dict(kin.get("lock_joints") or {})
    locks.update(dict.fromkeys(("base_x", "base_y", "base_theta"), 0.))
    locks.update({f"torso_{i}": float(q) for i, q in enumerate(torso_joints(h))})
    locks.update({f"{idle}_arm_{i}": float(q) for i, q in enumerate(finite_vector(idle_arm, 7))})
    locks.update({f"head_{i}": float(q) for i, q in enumerate(finite_vector(head, 2))})
    for s, q in zip(("l", "r"), finite_vector(grippers, 2)):
        if not -.05 <= q <= 0:
            raise ValueError("非法夹爪状态")
        locks[f"gripper_finger_{s}1"] = float(q)
        locks[f"gripper_finger_{s}2"] = float(-q)
    for j in active:
        locks.pop(j, None)
    kin["lock_joints"] = locks
    kin["ee_link"] = f"ee_{side}_tcp"
    kin["link_names"] = list(dict.fromkeys(kin.get("link_names", []) + ["link_head_2"]))
    cspace = kin["cspace"]
    original = cspace["joint_names"]
    indexes = [original.index(j) for j in active]
    for key, value in list(cspace.items()):
        if isinstance(value, list) and len(value) == len(original):
            cspace[key] = [value[i] for i in indexes]
    cspace["joint_names"] = active
    return cfg


def to_base_frame(world_pose, base_world_pose):
    world_pose, base_world_pose = np.asarray(world_pose), np.asarray(base_world_pose)
    for pose in (world_pose, base_world_pose):
        if pose.shape != (4, 4) or not np.isfinite(pose).all():
            raise ValueError("位姿需要有限 4x4 矩阵")
        if not np.allclose(pose[3], [0, 0, 0, 1]):
            raise ValueError("非法齐次矩阵")
        if not np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-5):
            raise ValueError("非法旋转矩阵")
        if not np.isclose(np.linalg.det(pose[:3, :3]), 1):
            raise ValueError("非法旋转行列式")
    return np.linalg.solve(base_world_pose, world_pose)


def successful_plan(result, trajectory):
    """cuRobo 失败会返回 [start_config]；必须检查实际 success，而不是非空。"""
    return (result.success is not None and bool(result.success.item()) and bool(trajectory))

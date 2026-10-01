"""严格 Pick 轨迹判定。缺证据视为基础设施异常，不视为不可达。"""
import numpy as np
from .protocol import finite_vector, torso_joints


def rotation_distance(a, b):
    return float(np.arccos(np.clip((np.trace(a.T @ b)-1)/2, -1., 1.)))


def validate_trace(samples, initial, thresholds, *, handle=False):
    required = {"time_s", "target_id", "base", "torso", "head", "idle_arm", "h", "illegal",
                "height_m", "support_contact", "selected_fingers", "relative_pose", "upright_rad", "finger_normal_forces_N", "contact_sampler_version"}
    if not samples or any(not required <= row.keys() for row in samples):
        return {"status": "infrastructure_error", "reason": "missing_trace_fields"}
    times = np.asarray([r["time_s"] for r in samples], dtype=float)
    if not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
        return {"status": "infrastructure_error", "reason": "invalid_sample_times"}
    if len(times) < 2 or np.max(np.diff(times)) > thresholds["max_sample_gap_s"]:
        return {"status": "infrastructure_error", "reason": "trace_gap"}
    for row in samples:
        if row['contact_sampler_version']!='force-aware-v2':
            return {'status':'infrastructure_error','reason':'unsupported_contact_sampler'}
        try:
            forces={int(k):float(v) for k,v in row['finger_normal_forces_N'].items()}
            fingers=set(int(k) for k in row['selected_fingers'])
        except (TypeError,ValueError,AttributeError):
            return {'status':'infrastructure_error','reason':'invalid_contact_forces'}
        if set(forces)!=fingers or any(not np.isfinite(v) or v<=1e-6 for v in forces.values()):
            return {'status':'infrastructure_error','reason':'invalid_contact_forces'}
    # 不只看末尾:全程检查底盘、躯干、闲置臂、头和禁止碰撞。
    b0 = finite_vector(initial["base"], 3)
    for row in samples:
        b = finite_vector(row["base"], 3)
        yaw = np.arctan2(np.sin(b[2]-b0[2]), np.cos(b[2]-b0[2]))
        if np.linalg.norm(b[:2]-b0[:2]) > thresholds["base_translation_m"] or abs(yaw) > thresholds["base_yaw_rad"]:
            return {"status": "executed_failure", "reason": "base_drift"}
        for name, reference, size, tol in (
            ("torso", torso_joints(row["h"]), 6, "torso_error_rad"),
            ("head", initial["head"], 2, "head_error_rad"),
            ("idle_arm", initial["idle_arm"], 7, "idle_arm_error_rad"),
        ):
            if np.max(np.abs(finite_vector(row[name], size)-finite_vector(reference, size))) > thresholds[tol]:
                return {"status": "executed_failure", "reason": name+"_protocol"}
        if row["target_id"] != initial["target_id"]:
            return {"status": "executed_failure", "reason": "wrong_target"}
        if row["illegal"]:
            return {"status": "executed_failure", "reason": "illegal_collision"}
        if not np.isfinite([row["height_m"], row["upright_rad"]]).all():
            return {"status": "infrastructure_error", "reason": "nonfinite_state"}
        pose = np.asarray(row["relative_pose"], dtype=float)
        if pose.shape != (4, 4) or not np.isfinite(pose).all():
            return {"status": "infrastructure_error", "reason": "invalid_relative_pose"}
        if (not np.allclose(pose[3], [0,0,0,1]) or
            not np.allclose(pose[:3,:3].T @ pose[:3,:3], np.eye(3), atol=1e-5) or
            not np.isclose(np.linalg.det(pose[:3,:3]), 1)):
            return {"status": "infrastructure_error", "reason": "invalid_relative_rotation"}
        if handle and (row["upright_rad"] > thresholds["upright_rad"] or row.get("forbidden_part_contact") is not False or row.get("pregrasp_motion_violation") is not False):
            return {"status": "executed_failure", "reason": "handle_semantics"}
    # 连续持有至少 2 s，最后一次断裂后重新计时。相对位姿以窗口首帧为基准。
    start, reference = None, None
    for row in samples:
        fingers = set(row["selected_fingers"])
        held = (fingers == set(initial["selected_fingers"]) and len(fingers) == 2 and
                row["height_m"]-initial["height_m"] >= thresholds["lift_m"] and
                row["support_contact"] is False)
        if handle:
            held = held and set(row.get("handle_fingers", [])) == fingers
        if not held:
            start, reference = None, None
            continue
        pose = np.asarray(row["relative_pose"])
        if start is None:
            start, reference = row["time_s"], pose
        if (np.linalg.norm(pose[:3,3]-reference[:3,3]) > thresholds["relative_translation_m"] or
                rotation_distance(reference[:3,:3], pose[:3,:3]) > thresholds["relative_rotation_rad"]):
            start, reference = None, None
    if start is None or times[-1]-start < thresholds["hold_s"]-1e-9:
        return {"status": "executed_failure", "reason": "lift_contact_hold_or_slip"}
    return {"status": "verified_success", "reason": "strict_pick"}


def select_grasps(all_ids, allowed_ids, mode):
    """把手集合为空时停止；不允许回退到普通抓法。"""
    if mode not in ("ordinary", "handle"):
        raise ValueError("未知抓取模式")
    if mode == "handle" and not allowed_ids:
        raise ValueError("没有经确认的把手 grasp，停止 Case 2")
    if allowed_ids is None:
        return list(all_ids)
    return [i for i in all_ids if i in set(allowed_ids)]


def validate_motion_prefix(samples, initial, thresholds):
    """躯干调整阶段：检查实际联动，不把控制目标的跟踪延迟误当额外自由度。"""
    for row in samples:
        base=finite_vector(row['base'],3);reference=finite_vector(initial['base'],3)
        yaw=np.arctan2(np.sin(base[2]-reference[2]),np.cos(base[2]-reference[2]))
        if np.linalg.norm(base[:2]-reference[:2])>thresholds['base_translation_m'] or abs(yaw)>thresholds['base_yaw_rad']:
            return 'base_drift_during_torso_adjust'
        q=finite_vector(row['torso'],6);actual_h=q[1]
        expected=np.array([0.,actual_h,-2*actual_h,actual_h,0.,0.])
        if np.max(np.abs(q-expected))>thresholds['torso_error_rad']:
            return 'linkage_violation_during_torso_adjust'
        if row['illegal']:return 'collision_during_torso_adjust'
        if row.get('forbidden_part_contact') or row.get('pregrasp_motion_violation'):return 'handle_semantics_during_torso_adjust'
        if 'handle_fingers' in row and row['upright_rad']>thresholds['upright_rad']:return 'cup_tilt_during_torso_adjust'
        for key,size,tolerance in [('head',2,'head_error_rad'),('idle_arm',7,'idle_arm_error_rad')]:
            if np.max(np.abs(finite_vector(row[key],size)-finite_vector(initial[key],size)))>thresholds[tolerance]:
                return key+'_drift_during_torso_adjust'
    return None

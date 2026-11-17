"""Robot-independent rigid-object geometry shared by UWA and cooperative tasks.

Poses are homogeneous transforms (metres); twists are [vx,vy,vz,wx,wy,wz]
at the object origin, expressed in the common world frame. No ROS clients.
"""
import numpy as np
from scipy.spatial.transform import Rotation


def rigid(value):
    t = np.asarray(value, dtype=float)
    if (t.shape != (4, 4) or not np.isfinite(t).all()
            or not np.allclose(t[3], [0, 0, 0, 1], atol=1e-9, rtol=0)
            or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-8, rtol=0)
            or abs(np.linalg.det(t[:3, :3]) - 1) > 1e-8):
        raise ValueError('Expected a finite right-handed rigid transform')
    return t.copy()


def relative(world_object, world_tcp):
    return np.linalg.inv(rigid(world_object)) @ rigid(world_tcp)


def tcp_target(world_object, object_tcp, object_twist=None):
    obj, offset = rigid(world_object), rigid(object_tcp)
    target = obj @ offset
    twist = np.zeros(6) if object_twist is None else np.asarray(object_twist, float)
    if twist.shape != (6,) or not np.isfinite(twist).all():
        raise ValueError('Expected six finite world-frame twist components')
    return target, np.r_[twist[:3] + np.cross(twist[3:], target[:3, 3] - obj[:3, 3]), twist[3:]]


def pose_error(actual, expected):
    a, b = rigid(actual), rigid(expected)
    return float(np.linalg.norm(a[:3, 3] - b[:3, 3])), float(
        Rotation.from_matrix(a[:3, :3] @ b[:3, :3].T).magnitude())


def interpolate(start, end, fraction):
    a, b = rigid(start), rigid(end)
    if not np.isfinite(fraction) or not 0 <= fraction <= 1:
        raise ValueError('Interpolation fraction must lie in [0,1]')
    t = a.copy()
    t[:3, 3] += fraction * (b[:3, 3] - a[:3, 3])
    t[:3, :3] = (Rotation.from_rotvec(fraction * Rotation.from_matrix(
        b[:3, :3] @ a[:3, :3].T).as_rotvec()).as_matrix() @ a[:3, :3])
    return t


def common_scale(commands, limits):
    """One factor for every member; never clip one member independently."""
    if not commands or set(commands) != set(limits):
        raise ValueError('Every member needs command limits')
    scale = 1.
    for member, command in commands.items():
        q, bound = np.asarray(command, float), np.asarray(limits[member], float)
        if q.shape != bound.shape or not np.isfinite([q, bound]).all() or np.any(bound <= 0):
            raise ValueError('Invalid shared command/limits')
        moving = np.abs(q) > 1e-12
        if moving.any():
            scale = min(scale, float(np.min(bound[moving] / np.abs(q[moving]))))
    return scale, {member: np.asarray(command) * scale for member, command in commands.items()}

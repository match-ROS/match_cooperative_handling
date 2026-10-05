"""Pose calculations for initializing a virtual object from MuR TCPs."""

import math
import re


PAIR_RE = re.compile(r"^(mur620[a-d]):([lr])$")


def parse_tcp_pairs(value):
    pairs = []
    for item in value.split(","):
        item = item.strip()
        match = PAIR_RE.fullmatch(item)
        if not match:
            raise ValueError(f"Invalid TCP '{item}'; expected mur620a:l, for example")
        pair = match.groups()
        if pair not in pairs:
            pairs.append(pair)
    if not pairs:
        raise ValueError("Select at least one TCP")
    return pairs


def normalize_quat(quat):
    norm = math.sqrt(sum(value * value for value in quat))
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("Invalid zero or non-finite orientation")
    return tuple(value / norm for value in quat)


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return normalize_quat((
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ))


def rotate_vector(quat, vector):
    x, y, z, w = normalize_quat(quat)
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


def rpy_degrees_to_quat(values):
    roll, pitch, yaw = (math.radians(value) for value in values)
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return normalize_quat((
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ))


def average_pose(poses):
    if not poses:
        raise ValueError("Select at least one TCP")
    count = len(poses)
    position = tuple(sum(pose[0][axis] for pose in poses) / count for axis in range(3))
    reference = normalize_quat(poses[0][1])
    quat_sum = [0.0] * 4
    for _, quat in poses:
        quat = normalize_quat(quat)
        if sum(a * b for a, b in zip(reference, quat)) < 0:
            quat = tuple(-value for value in quat)
        for index, value in enumerate(quat):
            quat_sum[index] += value
    return position, normalize_quat(quat_sum)


def offset_pose(center, xyz, rpy_deg, frame):
    """Translate and rotate about the center using object or map axes."""
    if frame not in ("object", "world"):
        raise ValueError("Offset frame must be object or world")
    if not all(math.isfinite(value) for value in (*xyz, *rpy_deg)):
        raise ValueError("Offsets must be finite")
    position, orientation = center
    orientation = normalize_quat(orientation)
    rotation = rpy_degrees_to_quat(rpy_deg)
    translation = rotate_vector(orientation, xyz) if frame == "object" else xyz
    shifted = tuple(position[index] + translation[index] for index in range(3))
    rotated = (
        quat_multiply(orientation, rotation) if frame == "object"
        else quat_multiply(rotation, orientation)
    )
    return shifted, rotated


def relative_pose(world_from_object, world_from_tcp):
    object_position, object_quat = world_from_object
    tcp_position, tcp_quat = world_from_tcp
    inverse_quat = tuple(-value for value in object_quat[:3]) + (object_quat[3],)
    displacement = tuple(tcp_position[index] - object_position[index] for index in range(3))
    return rotate_vector(inverse_quat, displacement), quat_multiply(inverse_quat, tcp_quat)

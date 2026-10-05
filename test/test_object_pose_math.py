"""Multi-MuR TCP selection and local/world object offsets."""

import math

import pytest

from match_cooperative_handling.object_pose_math import (
    average_pose, offset_pose, parse_tcp_pairs, relative_pose,
    rpy_degrees_to_quat,
)


IDENTITY = (0.0, 0.0, 0.0, 1.0)


def test_parse_tcp_pairs_across_robots():
    assert parse_tcp_pairs("mur620a:l,mur620b:r,mur620a:l") == [
        ("mur620a", "l"), ("mur620b", "r")
    ]
    with pytest.raises(ValueError):
        parse_tcp_pairs("mur620a:l,bad:r")
    with pytest.raises(ValueError):
        parse_tcp_pairs("")


def test_center_and_exact_tcp():
    poses = [((0.0, 0.0, 0.0), IDENTITY), ((2.0, 4.0, 6.0), IDENTITY)]
    assert average_pose(poses) == ((1.0, 2.0, 3.0), IDENTITY)
    assert average_pose(poses[:1]) == poses[0]
    assert average_pose([poses[0], (poses[1][0], (0.0, 0.0, 0.0, -1.0))]) == (
        (1.0, 2.0, 3.0), IDENTITY
    )


def test_object_and_world_offsets_use_different_axes():
    yaw_90 = rpy_degrees_to_quat((0.0, 0.0, 90.0))
    center = ((1.0, 2.0, 0.0), yaw_90)
    local = offset_pose(center, (1.0, 0.0, 0.0), (0.0, 0.0, 0.0), "object")
    world = offset_pose(center, (1.0, 0.0, 0.0), (0.0, 0.0, 0.0), "world")
    assert local[0] == pytest.approx((1.0, 3.0, 0.0))
    assert world[0] == pytest.approx((2.0, 2.0, 0.0))

    roll_90 = (90.0, 0.0, 0.0)
    local_rotation = offset_pose(center, (0, 0, 0), roll_90, "object")[1]
    world_rotation = offset_pose(center, (0, 0, 0), roll_90, "world")[1]
    assert local_rotation != pytest.approx(world_rotation)


def test_relative_pose_reconstructs_tcp_after_offset():
    object_pose = ((1.0, 2.0, 0.0), rpy_degrees_to_quat((0, 0, 90)))
    tcp_pose = ((1.0, 3.0, 0.0), IDENTITY)
    position, orientation = relative_pose(object_pose, tcp_pose)
    assert position == pytest.approx((1.0, 0.0, 0.0))
    assert math.sqrt(sum(value * value for value in orientation)) == pytest.approx(1.0)

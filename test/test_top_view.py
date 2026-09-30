"""Geometry and missing-TF checks for the lightweight map view."""

import math
import os

import pytest
from geometry_msgs.msg import TransformStamped
from PyQt5 import QtWidgets
from tf2_ros import TransformException

from match_cooperative_handling.top_view import (
    ARM_LINKS,
    OBJECT_FRAME,
    Pose2D,
    TopViewPanel,
    ViewSnapshot,
    collect_snapshot,
    frame_names,
    pose_from_transform,
    scene_point,
)


NOW_NS = 10_000_000_000


def transform(frame, x, y, yaw=0.0, stamp_ns=NOW_NS):
    msg = TransformStamped()
    msg.header.frame_id = "map"
    msg.child_frame_id = frame
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
    msg.transform.translation.x = x
    msg.transform.translation.y = y
    msg.transform.rotation.z = math.sin(yaw / 2)
    msg.transform.rotation.w = math.cos(yaw / 2)
    return msg


class FakeBuffer:
    def __init__(self, transforms):
        self.transforms = transforms

    def lookup_transform(self, target, source, _time):
        assert target == "map"
        if source not in self.transforms:
            raise TransformException("no transform")
        return self.transforms[source]


@pytest.fixture(scope="module")
def app():
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def tagged_items(panel):
    return {
        item.data(0): item
        for item in panel._scene.items()
        if item.data(0) is not None
    }


@pytest.mark.parametrize("count", [1, 2, 4])
def test_selected_robot_positions_and_object(count, app):
    robots = tuple(f"mur620{letter}" for letter in "abcd"[:count])
    transforms = {}
    for index, robot in enumerate(robots):
        x, y = 2.0 * index, (-1.0) ** index
        transforms[f"{robot}/base_link"] = transform(
            f"{robot}/base_link", x, y, math.pi / 2 if index == 0 else 0
        )
        prefix = f"{robot}/UR10_l"
        transforms[f"{prefix}/virtual_object_target_tcp"] = transform(
            f"{prefix}/virtual_object_target_tcp", x + 0.8, y + 0.4
        )
        for link_index, link in enumerate(ARM_LINKS):
            frame = f"{prefix}/{link}"
            transforms[frame] = transform(frame, x + link_index * 0.1, y + 0.3)
    transforms[OBJECT_FRAME] = transform(OBJECT_FRAME, 0.5, -0.5)

    snapshot = collect_snapshot(FakeBuffer(transforms), NOW_NS, robots, ("l",))
    assert not snapshot.problems
    panel = TopViewPanel()
    panel.resize(1000, 400)
    panel.show()
    app.processEvents()
    panel.update_snapshot(snapshot)
    items = tagged_items(panel)
    assert "object" in items
    assert len([tag for tag in items if str(tag).startswith("robot:")]) == count
    assert len([tag for tag in items if str(tag).startswith("target:")]) == count
    first = items["robot:mur620a"]
    assert first.pos() == scene_point(Pose2D(0, 1, math.pi / 2))
    assert first.rotation() == pytest.approx(-90.0)
    panel.close()


def test_only_checked_sides_and_empty_robot_selection(app):
    poses = {
        "mur620a/base_link": Pose2D(0, 0, 0),
        "mur620a/UR10_l/virtual_object_target_tcp": Pose2D(1, 0, 0),
        "mur620a/UR10_r/virtual_object_target_tcp": Pose2D(-1, 0, 0),
        OBJECT_FRAME: Pose2D(0, 1, 0),
    }
    panel = TopViewPanel()
    panel.update_snapshot(ViewSnapshot(("mur620a",), ("l",), poses, {}))
    assert "target:mur620a:l" in tagged_items(panel)
    assert "target:mur620a:r" not in tagged_items(panel)
    panel.update_snapshot(ViewSnapshot((), (), poses, {}))
    assert not tagged_items(panel)
    assert panel.status.text() == "No MuR selected"


def test_missing_stale_and_static_transforms():
    names = frame_names(("mur620a",), ("r",))
    assert "mur620a/UR10_r/virtual_object_target_tcp" in names
    assert "mur620a/UR10_l/virtual_object_target_tcp" not in names
    assert frame_names((), ("l",)) == ()

    stale = transform("mur620a/base_link", 1, 2, stamp_ns=NOW_NS - 3_000_000_000)
    pose, problem = pose_from_transform(stale, NOW_NS)
    assert pose is None and "stale" in problem
    static = transform("mur620a/base_link", 1, 2, stamp_ns=0)
    pose, problem = pose_from_transform(static, NOW_NS)
    assert problem is None and pose == Pose2D(1, 2, 0)

    snapshot = collect_snapshot(
        FakeBuffer({"mur620a/base_link": stale}), NOW_NS, ("mur620a",), ()
    )
    assert "mur620a/base_link" not in snapshot.poses
    assert "stale" in snapshot.problems["mur620a/base_link"]
    assert OBJECT_FRAME in snapshot.problems
    assert scene_point(Pose2D(1, 2, 0)).y() == -200

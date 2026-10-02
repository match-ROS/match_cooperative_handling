"""Non-actuating checks for cooperative GUI command routing."""

from types import SimpleNamespace

import pytest

from match_cooperative_handling.cooperative_gui_module import CooperativeHandlingModule


class FakeBridge:
    def __init__(self):
        self.twists = []
        self.robot_names = []
        self.stopped = False

    def publish_object_twist(self, values):
        self.twists.append(tuple(values))

    def set_robot_names(self, names):
        self.robot_names.append(tuple(names))

    def shutdown(self):
        self.stopped = True

    def wait(self, _timeout):
        return True


class FakeViewWorker:
    def __init__(self):
        self.resets = 0

    def reset_tf_cache(self):
        self.resets += 1

    def shutdown(self):
        pass

    def wait(self, _timeout):
        return True


class FakeToggle:
    def __init__(self, checked):
        self.checked = checked

    def isChecked(self):
        return self.checked


class FakeContext:
    def __init__(self, sides=("l", "r")):
        self.sides = list(sides)
        self.started = []
        self.calls = []
        self.logs = []
        self.window = SimpleNamespace(
            processes={}, freedrive_active={}, remote_ws=lambda: "/remote/workspace",
            enable_cartesian_motion=lambda pairs, on_success: on_success(),
        )
        self.ros_worker = SimpleNamespace(
            call_trigger=lambda service, label: self.calls.append((service, label))
        )

    def selected_robots(self):
        return ["mur620a", "mur620b"]

    def selected_sides(self):
        return list(self.sides)

    def object_host(self):
        return "mur620a"

    def process_key(self, robot, name):
        return f"{robot}:{name}"

    def remote_command(self, robot, command):
        return f"ssh:{robot}:{command}"

    def remote_ros_command(self, robot, command):
        return f"ssh_ros:{robot}:{command}"

    def start_process(self, name, command, env=None, on_finished=None):
        self.started.append((name, command, on_finished))

    def append_log(self, text):
        self.logs.append(text)

    def robot_arm_pairs(self, sides=None):
        return [
            (robot, side)
            for robot in self.selected_robots()
            for side in (sides or self.sides)
        ]

    def arm_status(self, _robot, _side):
        return "armed"

    def ur_reverse_ready(self, _robot, _side):
        return True

    def ensure_ur_ready(self, **kwargs):
        kwargs["on_success"]()


@pytest.mark.parametrize("temporary_anchor", [False, True])
def test_remote_workflow_and_optional_map_anchor(temporary_anchor):
    context = FakeContext()
    module = CooperativeHandlingModule()
    module.context = context
    module.ros_bridge = FakeBridge()
    module.top_view_worker = FakeViewWorker()
    module.temporary_map_anchor_button = FakeToggle(temporary_anchor)

    module.start_object_nodes()
    name, cleanup, after_cleanup = context.started[0]
    assert name == "object_cleanup"
    assert "ssh:mur620a:" in cleanup
    assert "ssh:mur620b:" in cleanup
    after_cleanup(0, None)
    assert module.top_view_worker.resets == 1

    starts = context.started[1:]
    names = {name for name, _, _ in starts}
    assert ("mur620a:map_tf" in names) == temporary_anchor
    assert "mur620a:object_state" in names
    assert {
        f"{robot}:object_transform_{side}"
        for robot in ("mur620a", "mur620b")
        for side in ("l", "r")
    }.issubset(names)
    assert all(command.startswith("ssh_ros:") for _, command, _ in starts)

    module.set_from_tcp()
    module.set_object_center()
    module.set_current_offsets()
    module.start_tracking_log()
    assert any("set_virtual_object_from_tcp.py" in command and "-p arm:=r" in command
               for _, command, _ in context.started)
    assert any("set_virtual_object_from_manipulators.py" in command
               for _, command, _ in context.started)
    assert sum("set_relative_pose_from_current_object.py" in command
               for _, command, _ in context.started) == 2
    assert sum("/remote/workspace/src/match_cooperative_handling/logs/tracking" in command
               for _, command, _ in context.started) == 2

    module.start_motion()
    assert len([service for service, _ in context.calls if service.endswith("/start")]) == 4
    module.start_demo("safe_wiggle", 0.05, 0.05, 5.0, 0.02, 0.1, 1)
    assert any("virtual_object_demo_runner" in command and command.startswith("ssh_ros:")
               for _, command, _ in context.started)
    module.stop_motion()
    assert len([service for service, _ in context.calls if service.endswith("/stop")]) == 4
    module.on_shutdown()
    assert module.ros_bridge.stopped
    assert context.started[-1][0] == "object_cleanup"
    context.started[-1][2](0, None)
    assert module.top_view_worker.resets == 2


def test_set_from_tcp_requires_selected_arm():
    context = FakeContext(sides=())
    module = CooperativeHandlingModule()
    module.context = context
    module.ros_bridge = FakeBridge()
    module.set_from_tcp()
    assert not context.started
    assert any("no arm selected" in line for line in context.logs)


def test_start_waits_for_controller_activation():
    context = FakeContext()
    pending = []
    context.window.enable_cartesian_motion = lambda pairs, on_success: pending.append(on_success)
    module = CooperativeHandlingModule()
    module.context = context
    module.ros_bridge = FakeBridge()
    module.start_motion()
    assert len(pending) == 1
    assert not context.calls
    pending[0]()
    assert len([service for service, _ in context.calls if service.endswith("/start")]) == 4

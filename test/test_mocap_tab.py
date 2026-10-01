"""Non-actuating Mocap tab controls and freeze feedback."""

import os
from types import SimpleNamespace

import pytest
from PyQt5 import QtCore, QtWidgets

from match_cooperative_handling import cooperative_mocap_tab


class FakeMonitor(QtCore.QObject):
    snapshot = QtCore.pyqtSignal(object)
    error = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.stopped = False

    def start(self):
        pass

    def shutdown(self):
        self.stopped = True

    def wait(self, _timeout):
        return True


class FakeRosWorker(QtCore.QObject):
    set_bool_result = QtCore.pyqtSignal(str, bool, bool, str)

    def __init__(self):
        super().__init__()
        self.calls = []

    def call_set_bool(self, service, enabled):
        self.calls.append((service, enabled))


class FakeProcess(QtCore.QObject):
    started = QtCore.pyqtSignal()

    def __init__(self):
        super().__init__()
        self.running = True

    def state(self):
        return QtCore.QProcess.Running if self.running else QtCore.QProcess.NotRunning

    def terminate(self):
        self.running = False

    def kill(self):
        self.running = False

    def waitForFinished(self, _timeout):
        return True


class FakeContext:
    def __init__(self):
        self.selected = ["mur620a", "mur620b"]
        self.ros_worker = FakeRosWorker()
        self.window = SimpleNamespace(processes={}, ros_worker=self.ros_worker)
        self.started = []
        self.logs = []

    def checked_robots(self):
        return self.selected

    def start_process(self, name, command, on_finished=None):
        self.started.append((name, command, on_finished))
        self.window.processes[name] = FakeProcess()

    def append_log(self, message):
        self.logs.append(message)


@pytest.fixture(scope="module")
def app():
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def test_start_stop_and_freeze_selected_robots(monkeypatch, app):
    monkeypatch.setattr(cooperative_mocap_tab, "MocapMonitor", FakeMonitor)
    context = FakeContext()
    resets = []
    tab = cooperative_mocap_tab.CooperativeMocapTab(
        context, reset_view_tf_cache=lambda: resets.append(True)
    )
    try:
        tab.start_driver()
        assert resets == [True]
        assert len(context.started) == 1
        name, command, finished = context.started[0]
        assert name == "mocap_driver"
        assert "publish_map_pose:=true" in command
        assert "publish_robot_tf:=true" in command
        assert "exec python3 -m match_mocap_ros2.qualisys_ssh_bridge" in command

        tab.set_frozen(True)
        assert not context.ros_worker.calls
        assert "no fresh map pose" in tab.action_status.text()

        snapshot = {
            robot: {"live": True, "hz": 100.0, "map_live": True, "frozen": False}
            for robot in ("mur620a", "mur620b", "mur620c", "mur620d")
        }
        tab._update_snapshot(snapshot)
        tab.set_frozen(True)
        assert set(context.ros_worker.calls) == {
            ("/qualisys/mur620a/freeze_localization", True),
            ("/qualisys/mur620b/freeze_localization", True),
        }
        assert not tab.freeze_button.isEnabled()
        for service, enabled in context.ros_worker.calls:
            context.ros_worker.set_bool_result.emit(service, enabled, True, "held")
        assert tab.freeze_button.isEnabled()
        assert "mur620a: OK" in tab.action_status.text()
        assert "mur620b: OK" in tab.action_status.text()

        tab.set_frozen(False)
        assert set(context.ros_worker.calls[-2:]) == {
            ("/qualisys/mur620a/freeze_localization", False),
            ("/qualisys/mur620b/freeze_localization", False),
        }
        tab.stop_driver()
        assert not context.window.processes[name].running
        finished(0, None)
        assert "Bridge stopped" in tab.driver_status.text()
    finally:
        tab.shutdown()
        assert tab.monitor.stopped
        tab.close()


def test_mocap_start_requires_temporary_anchor_to_stop(monkeypatch, app):
    monkeypatch.setattr(cooperative_mocap_tab, "MocapMonitor", FakeMonitor)
    context = FakeContext()
    anchor_process = FakeProcess()
    context.window.processes["mur620a:map_tf"] = anchor_process
    anchor_button = QtWidgets.QPushButton()
    anchor_button.setCheckable(True)
    anchor_button.setChecked(True)
    tab = cooperative_mocap_tab.CooperativeMocapTab(
        context, temporary_anchor_button=anchor_button
    )
    try:
        tab.start_driver()
        assert not context.started
        assert "temporary map anchor" in tab.driver_status.text()
        assert anchor_button.isChecked()

        anchor_process.terminate()
        tab.start_driver()
        assert len(context.started) == 1
        assert not anchor_button.isChecked()
    finally:
        tab.shutdown()
        tab.close()


def test_mocap_start_refuses_duplicate_external_map_source(monkeypatch, app):
    monkeypatch.setattr(cooperative_mocap_tab, "MocapMonitor", FakeMonitor)
    context = FakeContext()
    tab = cooperative_mocap_tab.CooperativeMocapTab(context)
    try:
        tab._update_snapshot({
            robot: {"live": True, "hz": 100.0, "map_live": True, "frozen": False}
            for robot in ("mur620a", "mur620b", "mur620c", "mur620d")
        })
        assert "External map poses" in tab.driver_status.text()
        tab.start_driver()
        assert not context.started
        assert "duplicate bridge" in context.logs[-1]
    finally:
        tab.shutdown()
        tab.close()

"""Exercise the complete Qt selection field without starting ROS or SSH."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5 import QtWidgets

from match_mur_gui import base_gui
from match_cooperative_handling import cooperative_gui_module as cooperative
from match_cooperative_handling import cooperative_mocap_tab as mocap


class _Signal:
    def connect(self, _callback):
        pass


class _Worker:
    def __init__(self, *_args):
        for name in (
            "log", "status", "snapshot", "error", "freedrive_status",
            "battery_status", "set_bool_result",
        ):
            setattr(self, name, _Signal())

    def start(self):
        pass

    def shutdown(self):
        pass

    def wait(self, _timeout):
        return True

    def set_robot_names(self, _names):
        pass

    def set_selection(self, _robots, _sides):
        pass


def test_virtual_object_field_routes_tcp_center_and_offset(monkeypatch, tmp_path):
    monkeypatch.setattr(base_gui, "RosWorker", _Worker)
    monkeypatch.setattr(cooperative, "CooperativeRosBridge", _Worker)
    monkeypatch.setattr(cooperative, "TopViewRosWorker", _Worker)
    monkeypatch.setattr(mocap, "MocapMonitor", _Worker)
    monkeypatch.setattr(
        base_gui.MurBaseGui, "_create_gui_log_file",
        lambda self: str(tmp_path / "gui.log"),
    )
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = base_gui.MurBaseGui(modules=[cooperative.CooperativeHandlingModule()])
    module = window.modules[0]
    window.stop_managed_processes = lambda: None
    module.stop_object_nodes = lambda **_kwargs: None
    commands = []
    window.start_process = lambda name, command, **_kwargs: commands.append((name, command))
    window.robot_checks["mur620d"].setChecked(False)
    window.robot_checks["mur620a"].setChecked(True)
    window.robot_checks["mur620b"].setChecked(True)
    window.resize(1280, 1000)
    window.show()
    app.processEvents()

    panel = module.object_panel
    assert panel.title() == "Set Virtual Object"
    assert panel.selected_pairs() == [
        ("mur620a", "l"), ("mur620a", "r"),
        ("mur620b", "l"), ("mur620b", "r"),
    ]
    assert not window.module_tabs.currentWidget().horizontalScrollBar().isVisible()

    panel.pair_rows[("mur620b", "l")].findChild(QtWidgets.QPushButton).click()
    assert commands[-1][0] == "mur620a:set_virtual_object"
    assert "-p tcp_pairs:=mur620b:l" in commands[-1][1]
    panel.pair_checks[("mur620b", "r")].setChecked(False)
    panel.center_button.click()
    assert "-p tcp_pairs:=mur620a:l,mur620a:r,mur620b:l" in commands[-1][1]

    dialog = cooperative.ObjectOffsetDialog(window)
    dialog.xyz[0].setValue(0.125)
    dialog.rpy[2].setValue(15.0)
    dialog.frame.setCurrentIndex(1)
    assert dialog.values() == ((0.125, 0.0, 0.0), (0.0, 0.0, 15.0), "world")
    dialog.close()

    monkeypatch.setattr(
        cooperative.ObjectOffsetDialog, "exec_", lambda self: QtWidgets.QDialog.Accepted
    )
    monkeypatch.setattr(
        cooperative.ObjectOffsetDialog, "values",
        lambda self: ((0.1, 0.2, -0.3), (10.0, 20.0, 30.0), "world"),
    )
    panel.offset_button.click()
    assert "-p offset_frame:=world" in commands[-1][1]
    assert "-p offset_z:=-0.300000" in commands[-1][1]

    window.arm_r.setChecked(False)
    app.processEvents()
    assert panel.selected_pairs() == [("mur620a", "l"), ("mur620b", "l")]
    panel.pair_checks[("mur620a", "l")].setChecked(False)
    panel.pair_checks[("mur620b", "l")].setChecked(False)
    assert not panel.center_button.isEnabled()
    assert not panel.offset_button.isEnabled()
    panel.pair_checks[("mur620b", "l")].setChecked(True)
    panel.center_button.click()
    assert "-p tcp_pairs:=mur620b:l" in commands[-1][1]
    window.close()

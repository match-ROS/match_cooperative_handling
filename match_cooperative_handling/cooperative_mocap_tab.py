"""Compact Qualisys controls for the Cooperative Handling GUI."""

import os
import shlex

from PyQt5 import QtCore, QtWidgets

from match_mocap_gui.mocap_gui_module import MocapMonitor
from match_mur_gui.base_gui import ROBOTS, WS


PROCESS_NAME = "mocap_driver"


class CooperativeMocapTab(QtWidgets.QWidget):
    """Control the local bridge and hold selected MuR map localizations."""

    def __init__(
        self, context, parent=None, temporary_anchor_button=None, reset_view_tf_cache=None
    ):
        super().__init__(parent)
        self.context = context
        self.temporary_anchor_button = temporary_anchor_button
        self.reset_view_tf_cache = reset_view_tf_cache
        self.last_snapshot = {}
        self.pending = set()
        self.results = []
        layout = QtWidgets.QVBoxLayout(self)

        bridge_row = QtWidgets.QHBoxLayout()
        self.start_button = QtWidgets.QPushButton("Start Mocap")
        self.start_button.clicked.connect(self.start_driver)
        bridge_row.addWidget(self.start_button)
        self.stop_button = QtWidgets.QPushButton("Stop Mocap")
        self.stop_button.clicked.connect(self.stop_driver)
        bridge_row.addWidget(self.stop_button)
        layout.addLayout(bridge_row)

        self.driver_status = QtWidgets.QLabel("Bridge stopped")
        layout.addWidget(self.driver_status)

        freeze_row = QtWidgets.QHBoxLayout()
        self.freeze_button = QtWidgets.QPushButton("Freeze selected MuRs")
        self.freeze_button.setToolTip(
            "Hold each checked MuR's last fresh map pose and continue publishing it"
        )
        self.freeze_button.clicked.connect(lambda: self.set_frozen(True))
        freeze_row.addWidget(self.freeze_button)
        self.resume_button = QtWidgets.QPushButton("Resume selected MuRs")
        self.resume_button.setToolTip(
            "Resume live Qualisys localization only when a fresh map pose is available"
        )
        self.resume_button.clicked.connect(lambda: self.set_frozen(False))
        freeze_row.addWidget(self.resume_button)
        layout.addLayout(freeze_row)

        self.action_status = QtWidgets.QLabel(
            "Freeze before an object hides the cameras; resume after visibility returns."
        )
        self.action_status.setWordWrap(True)
        layout.addWidget(self.action_status)

        self.table = QtWidgets.QTableWidget(len(ROBOTS), 4)
        self.table.setHorizontalHeaderLabels(("MuR", "Raw", "Map", "Localization"))
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.table.setMaximumHeight(170)
        for row, robot in enumerate(ROBOTS):
            self.table.setItem(row, 0, QtWidgets.QTableWidgetItem(robot))
            for column in range(1, 4):
                self.table.setItem(row, column, QtWidgets.QTableWidgetItem("—"))
        layout.addWidget(self.table)
        layout.addStretch(1)
        hint = QtWidgets.QLabel(
            "Hold affects /qualisys_map poses and map → MuR TF. "
            "Raw /qualisys poses remain available for diagnosis. "
            "Stopping the bridge also stops held TF publication."
        )
        hint.setWordWrap(True)
        layout.addWidget(hint)

        self.context.window.ros_worker.set_bool_result.connect(self._on_freeze_result)
        self.update_selection()
        self.monitor = MocapMonitor()
        self.monitor.snapshot.connect(self._update_snapshot)
        self.monitor.error.connect(context.append_log)
        self.monitor.start()

    def update_selection(self):
        selected = set(self.context.checked_robots())
        for row, robot in enumerate(ROBOTS):
            self.table.setRowHidden(row, robot not in selected)
        self.freeze_button.setEnabled(bool(selected) and not self.pending)
        self.resume_button.setEnabled(bool(selected) and not self.pending)
        if not selected:
            self.action_status.setText("Select at least one MuR to freeze its localization.")

    def _update_snapshot(self, snapshot):
        self.last_snapshot = snapshot
        for row, robot in enumerate(ROBOTS):
            state = snapshot[robot]
            raw = f"{state['hz']:.0f} Hz" if state['live'] else "Missing"
            frozen = state['frozen']
            mapped = "Held" if frozen and state['map_live'] else (
                "Live" if state['map_live'] else "Missing"
            )
            localization = (
                "FROZEN" if frozen is True else
                "Following" if frozen is False else "Unknown"
            )
            for column, value in enumerate((raw, mapped, localization), start=1):
                self.table.item(row, column).setText(value)
        process = self.context.window.processes.get(PROCESS_NAME)
        managed = process is not None and process.state() != QtCore.QProcess.NotRunning
        if not managed and any(state["map_live"] for state in snapshot.values()):
            self.driver_status.setText("External map poses are live")
        elif not managed and self.driver_status.text() == "External map poses are live":
            self.driver_status.setText("Bridge stopped")

    def start_driver(self):
        active_anchor = any(
            (name == "map_tf" or name.endswith(":map_tf"))
            and process.state() != QtCore.QProcess.NotRunning
            for name, process in self.context.window.processes.items()
        )
        if active_anchor:
            self.driver_status.setText("Stop the temporary map anchor before Mocap")
            self.context.append_log(
                "[mocap] Refusing start: temporary map anchor is still publishing"
            )
            return
        if self.temporary_anchor_button is not None:
            self.temporary_anchor_button.setChecked(False)
        process = self.context.window.processes.get(PROCESS_NAME)
        if process is not None and process.state() != QtCore.QProcess.NotRunning:
            self.context.append_log("[mocap] GUI-managed bridge is already running")
            return
        if any(state.get("map_live", False) for state in self.last_snapshot.values()):
            self.driver_status.setText("External map poses are already live")
            self.context.append_log(
                "[mocap] Refusing duplicate bridge: external map poses are live"
            )
            return
        if self.reset_view_tf_cache is not None:
            self.reset_view_tf_cache()
        install_setup = os.path.join(WS, "install", "setup.bash")
        command = (
            "source /opt/ros/jazzy/setup.bash && "
            f"source {shlex.quote(install_setup)} && "
            "export PYTHONUNBUFFERED=1 && "
            "exec python3 -m match_mocap_ros2.qualisys_ssh_bridge "
            "--ros-args -p publish_map_pose:=true -p publish_robot_tf:=true"
        )
        self.driver_status.setText("Bridge starting…")
        self.context.append_log("[mocap] Starting local Qualisys bridge with robot TF")
        self.context.start_process(PROCESS_NAME, command, on_finished=self._driver_finished)
        process = self.context.window.processes.get(PROCESS_NAME)
        if process is not None:
            process.started.connect(lambda: self.driver_status.setText("Bridge running"))

    def _driver_finished(self, code, _status):
        self.driver_status.setText(f"Bridge stopped (exit {code})")
        self.action_status.setText("Bridge stopped; held map TF is no longer published.")

    def stop_driver(self):
        process = self.context.window.processes.get(PROCESS_NAME)
        if process is None or process.state() == QtCore.QProcess.NotRunning:
            self.context.append_log("[mocap] No GUI-managed bridge is running")
            self.driver_status.setText("Bridge stopped")
            return
        self.context.append_log("[mocap] Stopping bridge; held map TF will also stop")
        process.terminate()
        if not process.waitForFinished(2000):
            process.kill()
            process.waitForFinished(2000)
        self.driver_status.setText("Bridge stopped")

    def set_frozen(self, enabled):
        robots = self.context.checked_robots()
        if not robots:
            self.action_status.setText("Select at least one MuR.")
            return
        if enabled:
            missing = [
                robot for robot in robots
                if not self.last_snapshot.get(robot, {}).get("map_live", False)
            ]
            if missing:
                self.action_status.setText(
                    "Cannot freeze: no fresh map pose for " + ", ".join(missing)
                )
                self.context.append_log("[mocap] " + self.action_status.text())
                return
        self.results = []
        self.pending = {
            f"/qualisys/{robot}/freeze_localization" for robot in robots
        }
        self.action_status.setText("Freezing…" if enabled else "Resuming…")
        self.update_selection()
        for service in tuple(self.pending):
            self.context.window.ros_worker.call_set_bool(service, enabled)

    def _on_freeze_result(self, service, enabled, success, message):
        if service not in self.pending:
            return
        self.pending.remove(service)
        robot = service.split("/")[2]
        result = f"{robot}: {'OK' if success else 'FAILED'} ({message})"
        self.results.append(result)
        self.context.append_log(f"[mocap] {'Freeze' if enabled else 'Resume'} {result}")
        if not self.pending:
            self.action_status.setText("; ".join(self.results))
            self.update_selection()

    def shutdown(self):
        self.monitor.shutdown()
        self.monitor.wait(2000)

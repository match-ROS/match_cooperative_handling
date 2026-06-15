#!/usr/bin/env python3
"""Local PyQt GUI for the cooperative handling virtual object layer."""

import os
import shlex
import signal
import sys
import threading
from functools import partial

from PyQt5 import QtCore, QtGui, QtWidgets

import rclpy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger


WS = os.environ.get("WS", "/home/rosmatch/colcon_ws")
HARDWARE_SCRIPT = os.path.join(
    WS, "src", "match_mobile_robotics_jazzy", "start_mur620_hardware_logged.sh"
)
PACKAGE = "match_cooperative_handling"
SIDES = {"r": "UR10_r", "l": "UR10_l"}


def setup_prefix():
    return (
        "source /opt/ros/jazzy/setup.bash && "
        f"source {shlex.quote(os.path.join(WS, 'install', 'setup.bash'))} && "
        f"export ROS_DOMAIN_ID={shlex.quote(os.environ.get('ROS_DOMAIN_ID', '62'))} && "
        "export ROS2CLI_NO_DAEMON=1 && "
        "export PYTHONUNBUFFERED=1 && "
        "export RCUTILS_LOGGING_BUFFERED_STREAM=0 && "
    )


class RosWorker(QtCore.QThread):
    log = QtCore.pyqtSignal(str)
    status = QtCore.pyqtSignal(str, str)

    def __init__(self, robot_name="mur620d"):
        super().__init__()
        self.robot_name = robot_name
        self._node = None
        self._object_twist_pub = None
        self._status_subs = []
        self._ready = threading.Event()
        self._stop = threading.Event()
        self._lock = threading.Lock()

    def run(self):
        rclpy.init(args=None)
        self._node = rclpy.create_node("cooperative_handling_gui")
        self._object_twist_pub = self._node.create_publisher(
            TwistStamped, "/virtual_object/object_twist_cmd", 10
        )
        self._configure_status_subscriptions(self.robot_name)
        self._ready.set()
        self.log.emit("[ros] GUI ROS helper started")
        while rclpy.ok() and not self._stop.is_set():
            rclpy.spin_once(self._node, timeout_sec=0.05)
        self.publish_object_twist([0.0] * 6)
        self._node.destroy_node()
        rclpy.shutdown()

    def shutdown(self):
        self._stop.set()

    def set_robot_name(self, robot_name):
        self.robot_name = robot_name
        if self._ready.wait(timeout=1.0):
            self._configure_status_subscriptions(robot_name)

    def _configure_status_subscriptions(self, robot_name):
        with self._lock:
            if self._node is None:
                return
            for sub in self._status_subs:
                self._node.destroy_subscription(sub)
            self._status_subs = []
            for side, prefix in SIDES.items():
                topic = f"/{robot_name}/{prefix}/virtual_object_tcp_transform_node/status"
                sub = self._node.create_subscription(
                    String,
                    topic,
                    partial(self._on_status, side),
                    10,
                )
                self._status_subs.append(sub)

    def _on_status(self, side, msg):
        self.status.emit(side, msg.data)

    def call_trigger(self, service_name, label):
        if not self._ready.wait(timeout=1.0):
            self.log.emit(f"[ros] Cannot call {label}: ROS helper not ready")
            return
        with self._lock:
            client = self._node.create_client(Trigger, service_name)
        if not client.wait_for_service(timeout_sec=0.5):
            self.log.emit(f"[ros] {label}: service unavailable: {service_name}")
            return
        future = client.call_async(Trigger.Request())

        def done(done_future):
            try:
                result = done_future.result()
                self.log.emit(
                    f"[ros] {label}: success={result.success}, message='{result.message}'"
                )
            except Exception as exc:  # noqa: BLE001
                self.log.emit(f"[ros] {label}: failed: {exc}")

        future.add_done_callback(done)

    def publish_object_twist(self, values):
        if self._object_twist_pub is None:
            return
        msg = TwistStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = f"{self.robot_name}/base_link"
        msg.twist.linear.x = float(values[0])
        msg.twist.linear.y = float(values[1])
        msg.twist.linear.z = float(values[2])
        msg.twist.angular.x = float(values[3])
        msg.twist.angular.y = float(values[4])
        msg.twist.angular.z = float(values[5])
        self._object_twist_pub.publish(msg)


class ObjectJogDialog(QtWidgets.QDialog):
    def __init__(self, ros_worker, parent=None):
        super().__init__(parent)
        self.ros_worker = ros_worker
        self.setWindowTitle("Virtual Object Jog")
        self.setMinimumWidth(430)
        self.mode = "translation"
        self.active = [0.0] * 6
        self.linear_speed = QtWidgets.QDoubleSpinBox()
        self.linear_speed.setRange(0.001, 0.2)
        self.linear_speed.setDecimals(3)
        self.linear_speed.setSingleStep(0.005)
        self.linear_speed.setValue(0.01)
        self.angular_speed = QtWidgets.QDoubleSpinBox()
        self.angular_speed.setRange(0.01, 1.0)
        self.angular_speed.setDecimals(3)
        self.angular_speed.setSingleStep(0.05)
        self.angular_speed.setValue(0.1)
        self.mode_label = QtWidgets.QLabel("Mode: translation")
        self.mode_label.setAlignment(QtCore.Qt.AlignCenter)

        layout = QtWidgets.QVBoxLayout(self)
        speed_row = QtWidgets.QHBoxLayout()
        speed_row.addWidget(QtWidgets.QLabel("Linear m/s"))
        speed_row.addWidget(self.linear_speed)
        speed_row.addWidget(QtWidgets.QLabel("Angular rad/s"))
        speed_row.addWidget(self.angular_speed)
        layout.addLayout(speed_row)
        layout.addWidget(self.mode_label)

        grid = QtWidgets.QGridLayout()
        self._add_button(grid, "Y+", 0, 1, [0, 1, 0])
        self._add_button(grid, "X-", 1, 0, [-1, 0, 0])
        stop_button = QtWidgets.QPushButton("STOP")
        stop_button.setMinimumHeight(48)
        stop_button.clicked.connect(self.stop)
        grid.addWidget(stop_button, 1, 1)
        self._add_button(grid, "X+", 1, 2, [1, 0, 0])
        self._add_button(grid, "Y-", 2, 1, [0, -1, 0])
        self._add_button(grid, "Z+", 0, 3, [0, 0, 1])
        self._add_button(grid, "Z-", 2, 3, [0, 0, -1])
        layout.addLayout(grid)

        hint = QtWidgets.QLabel("Keys: arrows X/Y, PgUp/PgDn Z, M mode, Space stop")
        hint.setAlignment(QtCore.Qt.AlignCenter)
        layout.addWidget(hint)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(50)

    def _add_button(self, grid, text, row, column, vector):
        button = QtWidgets.QPushButton(text)
        button.setMinimumHeight(48)
        button.pressed.connect(partial(self._set_axis, vector))
        button.released.connect(self.stop)
        grid.addWidget(button, row, column)

    def _set_axis(self, vector):
        self.active = [0.0] * 6
        offset = 0 if self.mode == "translation" else 3
        speed = self.linear_speed.value() if self.mode == "translation" else self.angular_speed.value()
        for index, value in enumerate(vector):
            self.active[offset + index] = value * speed

    def _tick(self):
        self.ros_worker.publish_object_twist(self.active)

    def stop(self):
        self.active = [0.0] * 6
        self.ros_worker.publish_object_twist(self.active)

    def toggle_mode(self):
        self.mode = "rotation" if self.mode == "translation" else "translation"
        self.mode_label.setText(f"Mode: {self.mode}")
        self.stop()

    def keyPressEvent(self, event):
        key = event.key()
        if key == QtCore.Qt.Key_M:
            self.toggle_mode()
            return
        if key in (QtCore.Qt.Key_Space, QtCore.Qt.Key_Period):
            self.stop()
            return
        mapping = {
            QtCore.Qt.Key_Left: [-1, 0, 0],
            QtCore.Qt.Key_Right: [1, 0, 0],
            QtCore.Qt.Key_Up: [0, 1, 0],
            QtCore.Qt.Key_Down: [0, -1, 0],
            QtCore.Qt.Key_PageUp: [0, 0, 1],
            QtCore.Qt.Key_PageDown: [0, 0, -1],
        }
        if key in mapping:
            self._set_axis(mapping[key])
            return
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if event.key() in (
            QtCore.Qt.Key_Left,
            QtCore.Qt.Key_Right,
            QtCore.Qt.Key_Up,
            QtCore.Qt.Key_Down,
            QtCore.Qt.Key_PageUp,
            QtCore.Qt.Key_PageDown,
        ):
            self.stop()
            return
        super().keyReleaseEvent(event)

    def closeEvent(self, event):
        self.stop()
        super().closeEvent(event)


class CooperativeHandlingGui(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MuR Cooperative Handling")
        self.resize(1180, 780)
        self.processes = {}
        self.arm_status = {"r": "unknown", "l": "unknown"}

        self.ros_worker = RosWorker("mur620")
        self.ros_worker.log.connect(self.append_log)
        self.ros_worker.status.connect(self.update_arm_status)
        self.ros_worker.start()

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)

        top = QtWidgets.QHBoxLayout()
        root.addLayout(top)
        top.addWidget(self._build_robot_box())
        top.addWidget(self._build_options_box(), 1)
        top.addWidget(self._build_status_box())

        actions = QtWidgets.QHBoxLayout()
        root.addLayout(actions)
        for button in (
            self._button("Start Hardware", self.start_hardware),
            self._button("Start Object Nodes", self.start_object_nodes),
            self._button("Set From TCP", self.set_from_tcp),
            self._button("Home L", partial(self.move_home, "l")),
            self._button("Home R", partial(self.move_home, "r")),
            self._button("Open Object Jog", self.open_object_jog),
            self._button("Stop Managed Processes", self.stop_managed_processes),
        ):
            actions.addWidget(button)

        tools = QtWidgets.QHBoxLayout()
        root.addLayout(tools)
        for button in (
            self._button("Open RViz", self.open_rviz),
            self._button("Set Object Center", self.set_object_center),
            self._button("Set Current Offsets", self.set_current_offsets),
        ):
            tools.addWidget(button)
        tools.addStretch(1)

        self.terminal = QtWidgets.QPlainTextEdit()
        self.terminal.setReadOnly(True)
        self.terminal.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        self.terminal.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont))
        root.addWidget(self.terminal, 1)

        bottom = QtWidgets.QHBoxLayout()
        root.addLayout(bottom)
        bottom.addStretch(1)
        self.start_motion_button = QtWidgets.QPushButton("START MOTION")
        self.start_motion_button.setMinimumSize(220, 72)
        self.start_motion_button.setStyleSheet(
            "QPushButton { background: #1f9d55; color: white; font-size: 22px; font-weight: bold; }"
        )
        self.start_motion_button.clicked.connect(self.start_motion)
        bottom.addWidget(self.start_motion_button)
        self.stop_motion_button = QtWidgets.QPushButton("STOP MOTION")
        self.stop_motion_button.setMinimumSize(220, 72)
        self.stop_motion_button.setStyleSheet(
            "QPushButton { background: #c53030; color: white; font-size: 22px; font-weight: bold; }"
        )
        self.stop_motion_button.clicked.connect(self.stop_motion)
        bottom.addWidget(self.stop_motion_button)

        self.append_log(
            "[gui] Ready. START MOTION only arms virtual-object control; "
            "Home L/R uses MoveIt separately."
        )

    def _build_robot_box(self):
        box = QtWidgets.QGroupBox("Robot")
        layout = QtWidgets.QFormLayout(box)
        self.robot_combo = QtWidgets.QComboBox()
        self.robot_combo.addItems(["mur620a", "mur620b", "mur620c", "mur620d"])
        self.robot_combo.setCurrentText("mur620d")
        layout.addRow("Profile", self.robot_combo)
        self.ros_name_edit = QtWidgets.QLineEdit("mur620")
        self.ros_name_edit.editingFinished.connect(self.on_ros_name_changed)
        layout.addRow("ROS name", self.ros_name_edit)
        self.arm_r = QtWidgets.QCheckBox("UR10_r")
        self.arm_r.setChecked(True)
        self.arm_l = QtWidgets.QCheckBox("UR10_l")
        self.arm_l.setChecked(True)
        layout.addRow(self.arm_r)
        layout.addRow(self.arm_l)
        return box

    def _build_options_box(self):
        box = QtWidgets.QGroupBox("Launch Options")
        layout = QtWidgets.QGridLayout(box)
        self.opt_build = self._check("Build before launch", True)
        self.opt_integrated = self._check("Integrated controller", True)
        self.opt_ft = self._check("Use FT sensor", True)
        self.opt_require_wrench = self._check("Require wrench", False)
        self.opt_collision = self._check("Collision avoidance", True)
        self.opt_markers = self._check("Collision markers", False)
        self.opt_moveit = self._check("Launch MoveIt", True)
        self.moveit_speed_label = QtWidgets.QLabel("MoveIt speed: 20%")
        self.moveit_speed_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.moveit_speed_slider.setRange(1, 100)
        self.moveit_speed_slider.setValue(20)
        self.moveit_speed_slider.valueChanged.connect(self.update_moveit_speed_label)
        for idx, widget in enumerate(
            [
                self.opt_build,
                self.opt_integrated,
                self.opt_ft,
                self.opt_require_wrench,
                self.opt_collision,
                self.opt_markers,
                self.opt_moveit,
            ]
        ):
            layout.addWidget(widget, idx // 2, idx % 2)
        row = (len([
            self.opt_build,
            self.opt_integrated,
            self.opt_ft,
            self.opt_require_wrench,
            self.opt_collision,
            self.opt_markers,
            self.opt_moveit,
        ]) + 1) // 2
        layout.addWidget(self.moveit_speed_label, row, 0)
        layout.addWidget(self.moveit_speed_slider, row, 1)
        return box

    def _build_status_box(self):
        box = QtWidgets.QGroupBox("Motion Gate")
        layout = QtWidgets.QFormLayout(box)
        self.status_r = QtWidgets.QLabel("unknown")
        self.status_l = QtWidgets.QLabel("unknown")
        layout.addRow("UR10_r", self.status_r)
        layout.addRow("UR10_l", self.status_l)
        return box

    def _check(self, text, checked):
        check = QtWidgets.QCheckBox(text)
        check.setChecked(checked)
        return check

    def _button(self, text, callback):
        button = QtWidgets.QPushButton(text)
        button.clicked.connect(callback)
        return button

    def update_moveit_speed_label(self, value):
        self.moveit_speed_label.setText(f"MoveIt speed: {value}%")

    def moveit_velocity_scaling(self):
        return max(1, min(100, self.moveit_speed_slider.value())) / 100.0

    def robot_profile(self):
        return self.robot_combo.currentText()

    def robot_name(self):
        return self.ros_name_edit.text().strip() or "mur620"

    def selected_sides(self):
        sides = []
        if self.arm_r.isChecked():
            sides.append("r")
        if self.arm_l.isChecked():
            sides.append("l")
        return sides

    def on_ros_name_changed(self):
        self.ros_worker.set_robot_name(self.robot_name())
        self.arm_status = {"r": "unknown", "l": "unknown"}
        self.status_r.setText("unknown")
        self.status_l.setText("unknown")

    def update_arm_status(self, side, status):
        self.arm_status[side] = status
        (self.status_r if side == "r" else self.status_l).setText(status)

    def append_log(self, text):
        self.terminal.appendPlainText(text.rstrip())
        self.terminal.verticalScrollBar().setValue(self.terminal.verticalScrollBar().maximum())

    def start_process(self, name, command, env=None):
        if name in self.processes and self.processes[name].state() != QtCore.QProcess.NotRunning:
            self.append_log(f"[gui] {name} already running")
            return
        process = QtCore.QProcess(self)
        process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        if env:
            qenv = QtCore.QProcessEnvironment.systemEnvironment()
            for key, value in env.items():
                qenv.insert(key, str(value))
            process.setProcessEnvironment(qenv)
        process.readyReadStandardOutput.connect(
            lambda proc=process, tag=name: self._read_process_output(tag, proc)
        )
        process.finished.connect(
            lambda code, status, tag=name: self.append_log(
                f"[{tag}] finished exit_code={code}, status={int(status)}"
            )
        )
        self.processes[name] = process
        self.append_log(f"[{name}] $ {command}")
        process.start("bash", ["-lc", command])

    def _read_process_output(self, tag, process):
        data = bytes(process.readAllStandardOutput()).decode(errors="replace")
        for line in data.splitlines():
            self.append_log(f"[{tag}] {line}")

    def start_hardware(self):
        profile = self.robot_profile()
        args = [
            f"launch_ur_r:={'true' if self.arm_r.isChecked() else 'false'}",
            f"launch_ur_l:={'true' if self.arm_l.isChecked() else 'false'}",
            f"integrated_controller_enable_collision_avoidance:={'true' if self.opt_collision.isChecked() else 'false'}",
            f"integrated_controller_publish_collision_markers:={'true' if self.opt_markers.isChecked() else 'false'}",
            f"launch_moveit:={'true' if self.opt_moveit.isChecked() else 'false'}",
            "auto_switch_moveit_controllers:=true",
            "launch_moveit_rviz:=false",
        ]
        if self.opt_integrated.isChecked():
            args.extend([
                "launch_arm_velocity_safety:=false",
                "launch_jparse_idk:=false",
            ])
        env = {
            "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "62"),
            "ROBOT_PROFILE": profile,
            "BUILD_BEFORE_LAUNCH": "true" if self.opt_build.isChecked() else "false",
            "BUILD_PACKAGES": (
                "serial ewellix_driver mur_control mur_moveit_config "
                "mur_launch_hardware match_cooperative_handling"
            ),
            "INTEGRATED_CARTESIAN_ACTIVE": "true" if self.opt_integrated.isChecked() else "false",
            "INTEGRATED_CARTESIAN_USE_FT": "true" if self.opt_ft.isChecked() else "false",
            "INTEGRATED_CARTESIAN_REQUIRE_WRENCH": (
                "true" if self.opt_require_wrench.isChecked() else "false"
            ),
            "MOVEIT_WITH_INTEGRATED_CARTESIAN": (
                "true" if self.opt_moveit.isChecked() and self.opt_integrated.isChecked() else "false"
            ),
        }
        command = " ".join([shlex.quote(HARDWARE_SCRIPT)] + [shlex.quote(arg) for arg in args])
        self.start_process("hardware", command, env)

    def start_object_nodes(self):
        robot = self.robot_name()
        state_cmd = (
            setup_prefix()
            + "exec ros2 run match_cooperative_handling virtual_object_state_node --ros-args "
            + f"-p world_frame:={robot}/base_link"
        )
        self.start_process("object_state", state_cmd)
        for side in self.selected_sides():
            prefix = SIDES[side]
            transform_cmd = (
                setup_prefix()
                + "exec ros2 run match_cooperative_handling virtual_object_tcp_transform_node --ros-args "
                + f"-r __ns:=/{robot}/{prefix} "
                + f"-p robot_name:={robot} "
                + f"-p arm:={side}"
            )
            self.start_process(f"object_transform_{side}", transform_cmd)

    def set_from_tcp(self):
        side = "r" if self.arm_r.isChecked() else "l"
        robot = self.robot_name()
        cmd = (
            setup_prefix()
            + "exec ros2 run match_cooperative_handling set_virtual_object_from_tcp.py --ros-args "
            + f"-p robot_name:={robot} "
            + f"-p arm:={side}"
        )
        self.start_process(f"set_from_tcp_{side}", cmd)

    def set_object_center(self):
        sides = self.selected_sides()
        if len(sides) < 2:
            self.append_log("[gui] Refusing object center: select at least two manipulators")
            return
        robot = self.robot_name()
        arms = ",".join(sides)
        cmd = (
            setup_prefix()
            + "exec ros2 run match_cooperative_handling "
            + "set_virtual_object_from_manipulators.py --ros-args "
            + f"-p robot_name:={robot} "
            + f"-p arms:={arms} "
            + f"-p world_frame:={robot}/base_link"
        )
        self.append_log(
            f"[gui] Setting virtual object center from selected manipulators: {arms}"
        )
        self.start_process("set_object_center", cmd)

    def set_current_offsets(self):
        sides = self.selected_sides()
        if not sides:
            self.append_log("[gui] Refusing current offsets: select at least one manipulator")
            return
        robot = self.robot_name()
        arms = ",".join(sides)
        cmd = (
            setup_prefix()
            + "exec ros2 run match_cooperative_handling "
            + "set_relative_pose_from_current_object.py --ros-args "
            + f"-p robot_name:={robot} "
            + f"-p arms:={arms} "
            + f"-p world_frame:={robot}/base_link "
            + "-p object_frame:=virtual_object/base_link "
            + "-p max_distance:=2.0"
        )
        self.append_log(
            f"[gui] Setting current object-relative TCP offsets for: {arms}"
        )
        self.start_process("set_current_offsets", cmd)

    def open_object_jog(self):
        dialog = ObjectJogDialog(self.ros_worker, self)
        dialog.show()
        dialog.raise_()
        dialog.activateWindow()
        self._jog_dialog = dialog

    def open_rviz(self):
        robot = self.robot_name()
        config_path = (
            "$(ros2 pkg prefix mur_launch_hardware)"
            "/share/mur_launch_hardware/config/rviz/mur620d_moveit.rviz"
        )
        cmd = (
            setup_prefix()
            + "exec rviz2 "
            + f"-d {config_path} "
            + "--ros-args "
            + f"-r /tf:=/tf "
            + f"-r /tf_static:=/tf_static "
            + f"-p use_sim_time:=false"
        )
        self.append_log(
            f"[gui] Opening RViz for {robot}. MoveIt must be running for MotionPlanning."
        )
        self.start_process("rviz_moveit", cmd)

    def move_home(self, side):
        prefix = SIDES[side]
        if not self.opt_moveit.isChecked():
            self.append_log(
                f"[gui] Home {prefix}: Launch MoveIt is disabled. "
                "Enable it before starting hardware, or make sure MoveIt is already running."
            )
        self.append_log(
            f"[gui] Home {prefix}: stopping virtual-object motion gate first; "
            "START MOTION is not required for Home."
        )
        self.ros_worker.publish_object_twist([0.0] * 6)
        service = f"/{self.robot_name()}/{prefix}/virtual_object_tcp_transform_node/stop"
        self.ros_worker.call_trigger(service, f"disarm before home {prefix}")
        QtCore.QTimer.singleShot(500, partial(self._start_home_process, side))

    def _start_home_process(self, side):
        prefix = SIDES[side]
        robot = self.robot_name()
        cmd = (
            setup_prefix()
            + "exec ros2 run match_cooperative_handling move_arm_to_named_pose.py --ros-args "
            + f"-p robot_name:={robot} "
            + f"-p robot_profile:={self.robot_profile()} "
            + f"-p arm:={side} "
            + f"-p group:=UR_arm_{side} "
            + "-p named_pose:=Home_custom "
            + f"-p velocity_scaling:={self.moveit_velocity_scaling():.3f}"
        )
        self.start_process(f"home_{side}", cmd)

    def start_motion(self):
        sides = self.selected_sides()
        if not sides:
            self.append_log("[gui] Refusing start: no arm selected")
            return
        blocked = []
        for side in sides:
            status = self.arm_status.get(side, "unknown")
            if not (status == "ready" or status == "armed"):
                blocked.append(f"{SIDES[side]}={status}")
        if blocked:
            self.append_log("[gui] Refusing start: " + ", ".join(blocked))
            return
        for side in sides:
            service = f"/{self.robot_name()}/{SIDES[side]}/virtual_object_tcp_transform_node/start"
            self.ros_worker.call_trigger(service, f"start {SIDES[side]}")

    def stop_motion(self):
        self.ros_worker.publish_object_twist([0.0] * 6)
        for side in self.selected_sides() or ["r", "l"]:
            service = f"/{self.robot_name()}/{SIDES[side]}/virtual_object_tcp_transform_node/stop"
            self.ros_worker.call_trigger(service, f"stop {SIDES[side]}")

    def stop_managed_processes(self):
        for name, process in list(self.processes.items()):
            if process.state() == QtCore.QProcess.NotRunning:
                continue
            self.append_log(f"[gui] terminating {name}")
            process.terminate()
            if not process.waitForFinished(1500):
                process.kill()

    def closeEvent(self, event):
        self.stop_motion()
        self.stop_managed_processes()
        self.ros_worker.shutdown()
        self.ros_worker.wait(1500)
        super().closeEvent(event)


def main():
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QtWidgets.QApplication(sys.argv)
    window = CooperativeHandlingGui()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

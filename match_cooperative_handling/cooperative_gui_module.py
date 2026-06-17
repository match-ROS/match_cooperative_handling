"""Cooperative handling extension module for the shared MuR GUI."""

import os
import shlex
import threading
import time
from functools import partial

from PyQt5 import QtCore, QtWidgets

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Bool, String

from match_mur_gui.base_gui import MurGuiModule, ROBOTS, SIDES


WORLD_FRAME = "map"


class CooperativeRosBridge(QtCore.QThread):
    log = QtCore.pyqtSignal(str)
    status = QtCore.pyqtSignal(str, str, str)

    def __init__(self, robot_names=None):
        super().__init__()
        self.robot_names = list(robot_names or ["mur620d"])
        self._node = None
        self._object_twist_pub = None
        self._tracking_stop_pub = None
        self._status_subs = []
        self._ready = threading.Event()
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._owns_rclpy = False

    def run(self):
        deadline = time.monotonic() + 2.0
        while not rclpy.ok() and time.monotonic() < deadline:
            self.msleep(20)
        if not rclpy.ok():
            rclpy.init(args=None)
            self._owns_rclpy = True
        self._node = rclpy.create_node("cooperative_handling_gui_bridge")
        self._object_twist_pub = self._node.create_publisher(
            TwistStamped, "/virtual_object/object_twist_cmd", 10
        )
        self._tracking_stop_pub = self._node.create_publisher(
            Bool, "/cooperative_tracking_logger/stop", 10
        )
        self._configure_status_subscriptions(self.robot_names)
        self._ready.set()
        self.log.emit("[ros] Cooperative GUI bridge started")
        try:
            while rclpy.ok() and not self._stop.is_set():
                rclpy.spin_once(self._node, timeout_sec=0.05)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            if rclpy.ok():
                self.publish_object_twist([0.0] * 6)
            if self._node is not None:
                self._node.destroy_node()
                self._node = None
            if self._owns_rclpy and rclpy.ok():
                rclpy.shutdown()

    def shutdown(self):
        self._stop.set()

    def set_robot_names(self, robot_names):
        self.robot_names = list(robot_names or ["mur620d"])
        if self._ready.wait(timeout=1.0):
            self._configure_status_subscriptions(self.robot_names)

    def _configure_status_subscriptions(self, robot_names):
        with self._lock:
            if self._node is None:
                return
            for sub in self._status_subs:
                self._node.destroy_subscription(sub)
            self._status_subs = []
            for robot_name in robot_names:
                for side, prefix in SIDES.items():
                    topic = f"/{robot_name}/{prefix}/virtual_object_tcp_transform_node/status"
                    sub = self._node.create_subscription(
                        String,
                        topic,
                        partial(self._on_status, robot_name, side),
                        10,
                    )
                    self._status_subs.append(sub)

    def _on_status(self, robot_name, side, msg):
        self.status.emit(robot_name, side, msg.data)

    def publish_object_twist(self, values):
        if self._object_twist_pub is None or self._node is None:
            return
        msg = TwistStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = WORLD_FRAME
        msg.twist.linear.x = float(values[0])
        msg.twist.linear.y = float(values[1])
        msg.twist.linear.z = float(values[2])
        msg.twist.angular.x = float(values[3])
        msg.twist.angular.y = float(values[4])
        msg.twist.angular.z = float(values[5])
        self._object_twist_pub.publish(msg)

    def publish_tracking_stop(self):
        if self._tracking_stop_pub is None:
            return
        msg = Bool()
        msg.data = True
        for _ in range(5):
            self._tracking_stop_pub.publish(msg)
            self.msleep(20)


class ObjectJogDialog(QtWidgets.QDialog):
    def __init__(self, ros_bridge, parent=None):
        super().__init__(parent)
        self.ros_bridge = ros_bridge
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
        self.ros_bridge.publish_object_twist(self.active)

    def stop(self):
        self.active = [0.0] * 6
        self.ros_bridge.publish_object_twist(self.active)

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


class DemoDialog(QtWidgets.QDialog):
    def __init__(self, module, parent=None):
        super().__init__(parent)
        self.module = module
        self.setWindowTitle("Cooperative Handling Demos")
        self.setMinimumWidth(520)

        layout = QtWidgets.QVBoxLayout(self)
        demo_box = QtWidgets.QGroupBox("Demo")
        demo_layout = QtWidgets.QVBoxLayout(demo_box)
        self.demo_combo = QtWidgets.QComboBox()
        self.demo_combo.addItem("Safe Wiggle", "safe_wiggle")
        demo_layout.addWidget(self.demo_combo)
        catalog = QtWidgets.QLabel(
            "Prepared next: Rounded Rectangle, Ellipse/Figure Eight, Tilt Demo, Teach Workspace"
        )
        catalog.setWordWrap(True)
        demo_layout.addWidget(catalog)
        layout.addWidget(demo_box)

        params = QtWidgets.QGroupBox("Safe Wiggle Parameters")
        form = QtWidgets.QFormLayout(params)
        self.xy_amplitude = QtWidgets.QDoubleSpinBox()
        self.xy_amplitude.setRange(0.001, 0.2)
        self.xy_amplitude.setDecimals(3)
        self.xy_amplitude.setSingleStep(0.005)
        self.xy_amplitude.setValue(0.05)
        form.addRow("XY amplitude [m]", self.xy_amplitude)
        self.z_lift = QtWidgets.QDoubleSpinBox()
        self.z_lift.setRange(0.001, 0.2)
        self.z_lift.setDecimals(3)
        self.z_lift.setSingleStep(0.005)
        self.z_lift.setValue(0.05)
        form.addRow("Z lift [m]", self.z_lift)
        self.yaw_amplitude = QtWidgets.QDoubleSpinBox()
        self.yaw_amplitude.setRange(0.1, 30.0)
        self.yaw_amplitude.setDecimals(1)
        self.yaw_amplitude.setSingleStep(1.0)
        self.yaw_amplitude.setValue(5.0)
        form.addRow("Yaw amplitude [deg]", self.yaw_amplitude)
        self.linear_velocity = QtWidgets.QDoubleSpinBox()
        self.linear_velocity.setRange(0.001, 0.15)
        self.linear_velocity.setDecimals(3)
        self.linear_velocity.setSingleStep(0.005)
        self.linear_velocity.setValue(0.02)
        form.addRow("Linear velocity [m/s]", self.linear_velocity)
        self.angular_velocity = QtWidgets.QDoubleSpinBox()
        self.angular_velocity.setRange(0.01, 0.8)
        self.angular_velocity.setDecimals(3)
        self.angular_velocity.setSingleStep(0.05)
        self.angular_velocity.setValue(0.10)
        form.addRow("Angular velocity [rad/s]", self.angular_velocity)
        self.repetitions = QtWidgets.QSpinBox()
        self.repetitions.setRange(1, 20)
        self.repetitions.setValue(1)
        form.addRow("Repetitions", self.repetitions)
        layout.addWidget(params)

        hint = QtWidgets.QLabel(
            "Demos do not arm the robot. Press START MOTION first, then start a demo."
        )
        hint.setWordWrap(True)
        layout.addWidget(hint)

        buttons = QtWidgets.QHBoxLayout()
        start = QtWidgets.QPushButton("Start Demo")
        start.setMinimumHeight(42)
        start.clicked.connect(self.start_demo)
        stop = QtWidgets.QPushButton("Stop Demo")
        stop.setMinimumHeight(42)
        stop.clicked.connect(self.module.stop_demo)
        buttons.addWidget(start)
        buttons.addWidget(stop)
        layout.addLayout(buttons)

    def start_demo(self):
        self.module.start_demo(
            demo_name=self.demo_combo.currentData(),
            xy_amplitude=self.xy_amplitude.value(),
            z_lift=self.z_lift.value(),
            yaw_amplitude_deg=self.yaw_amplitude.value(),
            linear_velocity=self.linear_velocity.value(),
            angular_velocity=self.angular_velocity.value(),
            repetitions=self.repetitions.value(),
        )

    def closeEvent(self, event):
        self.module.stop_demo()
        super().closeEvent(event)


class CooperativeHandlingModule(MurGuiModule):
    def __init__(self):
        self.context = None
        self.ros_bridge = None
        self._jog_dialog = None
        self._demo_dialog = None

    def setup_ui(self, context):
        self.context = context
        self.ros_bridge = CooperativeRosBridge(context.selected_robots())
        self.ros_bridge.log.connect(context.append_log)
        self.ros_bridge.status.connect(context.set_arm_status)
        self.ros_bridge.start()

        context.add_action_button("Start Object Nodes", self.start_object_nodes)
        context.add_action_button("Set From TCP", self.set_from_tcp)
        context.add_action_button("Open Object Jog", self.open_object_jog)
        context.add_tool_button("Demos", self.open_demos)
        context.add_tool_button("Start Tracking Log", self.start_tracking_log)
        context.add_tool_button("Stop Tracking Log", self.stop_tracking_log)
        context.add_tool_button("Set Object Center", self.set_object_center)
        context.add_tool_button("Set Current Offsets", self.set_current_offsets)

        self.start_motion_button = QtWidgets.QPushButton("START MOTION")
        self.start_motion_button.setMinimumSize(220, 72)
        self.start_motion_button.setStyleSheet(
            "QPushButton { background: #1f9d55; color: white; font-size: 22px; font-weight: bold; }"
        )
        self.start_motion_button.clicked.connect(self.start_motion)
        context.add_bottom_widget(self.start_motion_button)

        self.stop_motion_button = QtWidgets.QPushButton("STOP MOTION")
        self.stop_motion_button.setMinimumSize(220, 72)
        self.stop_motion_button.setStyleSheet(
            "QPushButton { background: #c53030; color: white; font-size: 22px; font-weight: bold; }"
        )
        self.stop_motion_button.clicked.connect(self.stop_motion)
        context.add_bottom_widget(self.stop_motion_button)

        context.append_log(
            "[gui] Cooperative Handling module loaded. START MOTION only arms "
            "virtual-object control; Home L/R uses MoveIt separately."
        )

    def on_robot_selection_changed(self):
        if self.ros_bridge is not None:
            self.ros_bridge.set_robot_names(self.selected_robots())

    def selected_sides(self):
        return self.context.selected_sides()

    def selected_robots(self):
        return self.context.selected_robots()

    def object_host(self):
        return self.context.object_host()

    def process_key(self, robot, name):
        return self.context.process_key(robot, name)

    def remote_command(self, robot, command):
        return self.context.remote_command(robot, command)

    def remote_setup_prefix(self):
        return self.context.remote_setup_prefix()

    def start_process(self, name, command, env=None, on_finished=None):
        self.context.start_process(name, command, env=env, on_finished=on_finished)

    def append_log(self, text):
        self.context.append_log(text)

    def start_object_nodes(self):
        self.stop_demo()
        self.ros_bridge.publish_object_twist([0.0] * 6)
        self.stop_object_nodes(start_after_cleanup=True)

    def stop_object_nodes(self, start_after_cleanup=False):
        cleanup_process = self.context.window.processes.get("object_cleanup")
        if cleanup_process is not None and cleanup_process.state() != QtCore.QProcess.NotRunning:
            self.append_log("[gui] terminating previous object_cleanup")
            cleanup_process.terminate()
            if not cleanup_process.waitForFinished(1000):
                cleanup_process.kill()

        managed_names = ["map_tf", "object_state"]
        for robot in ROBOTS:
            managed_names.extend([
                self.process_key(robot, "map_tf"),
                self.process_key(robot, "object_state"),
            ])
            for side in SIDES:
                managed_names.append(self.process_key(robot, f"object_transform_{side}"))
        for name in managed_names:
            process = self.context.window.processes.get(name)
            if process is not None and process.state() != QtCore.QProcess.NotRunning:
                self.append_log(f"[gui] terminating {name}")
                process.terminate()
                if not process.waitForFinished(1000):
                    process.kill()

        cleanup_patterns = [
            "/match_cooperative_handling/[v]irtual_object_state_node",
            "match_cooperative_handling [v]irtual_object_state_node",
            "/match_cooperative_handling/[v]irtual_object_tcp_transform_node",
            "match_cooperative_handling [v]irtual_object_tcp_transform_node",
        ]
        cleanup_cmd = " ; ".join(
            f"pkill -TERM -f {shlex.quote(pattern)} 2>/dev/null || true"
            for pattern in cleanup_patterns
        )
        cleanup_cmd += " ; sleep 0.5 ; "
        cleanup_cmd += " ; ".join(
            f"pkill -KILL -f {shlex.quote(pattern)} 2>/dev/null || true"
            for pattern in cleanup_patterns
        )
        remote_cleanup = [self.remote_command(robot, cleanup_cmd) for robot in self.selected_robots()]
        if remote_cleanup:
            cleanup_cmd = " ; ".join(remote_cleanup)

        if start_after_cleanup:
            self.append_log("[gui] Restarting virtual object nodes with a clean slate")
            self.start_process(
                "object_cleanup",
                cleanup_cmd,
                on_finished=lambda _code, _status: self._start_object_nodes_after_cleanup(),
            )
        else:
            self.append_log("[gui] Stopping virtual object nodes")
            self.start_process("object_cleanup", cleanup_cmd)

    def _start_object_nodes_after_cleanup(self):
        object_host = self.object_host()
        self.ros_bridge.set_robot_names(self.selected_robots())
        map_cmd = (
            self.remote_setup_prefix()
            + f"exec ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 "
            + f"{WORLD_FRAME} {object_host}/base_link"
        )
        self.start_process(
            self.process_key(object_host, "map_tf"),
            self.remote_command(object_host, map_cmd),
        )
        state_cmd = (
            self.remote_setup_prefix()
            + "exec ros2 run match_cooperative_handling virtual_object_state_node --ros-args "
            + f"-p world_frame:={WORLD_FRAME} "
            + "-p rate:=500.0"
        )
        self.start_process(
            self.process_key(object_host, "object_state"),
            self.remote_command(object_host, state_cmd),
        )
        for robot in self.selected_robots():
            for side in self.selected_sides():
                prefix = SIDES[side]
                transform_cmd = (
                    self.remote_setup_prefix()
                    + "exec ros2 run match_cooperative_handling virtual_object_tcp_transform_node --ros-args "
                    + f"-r __ns:=/{robot}/{prefix} "
                    + f"-p robot_name:={robot} "
                    + f"-p arm:={side} "
                    + f"-p world_frame:={WORLD_FRAME} "
                    + "-p rate:=500.0"
                )
                self.start_process(
                    self.process_key(robot, f"object_transform_{side}"),
                    self.remote_command(robot, transform_cmd),
                )

    def set_from_tcp(self):
        side = "r" if "r" in self.selected_sides() else "l"
        robot = self.object_host()
        cmd = (
            self.remote_setup_prefix()
            + "exec ros2 run match_cooperative_handling set_virtual_object_from_tcp.py --ros-args "
            + f"-p robot_name:={robot} "
            + f"-p arm:={side} "
            + f"-p world_frame:={WORLD_FRAME}"
        )
        self.start_process(
            self.process_key(robot, f"set_from_tcp_{side}"),
            self.remote_command(robot, cmd),
        )

    def set_object_center(self):
        sides = self.selected_sides()
        if len(sides) < 2:
            self.append_log("[gui] Refusing object center: select at least two manipulators")
            return
        robot = self.object_host()
        arms = ",".join(sides)
        cmd = (
            self.remote_setup_prefix()
            + "exec ros2 run match_cooperative_handling "
            + "set_virtual_object_from_manipulators.py --ros-args "
            + f"-p robot_name:={robot} "
            + f"-p arms:={arms} "
            + f"-p world_frame:={WORLD_FRAME}"
        )
        self.append_log(
            f"[gui] Setting virtual object center on {robot} from selected manipulators: {arms}"
        )
        self.start_process(
            self.process_key(robot, "set_object_center"),
            self.remote_command(robot, cmd),
        )

    def set_current_offsets(self):
        sides = self.selected_sides()
        if not sides:
            self.append_log("[gui] Refusing current offsets: select at least one manipulator")
            return
        arms = ",".join(sides)
        for robot in self.selected_robots():
            cmd = (
                self.remote_setup_prefix()
                + "exec ros2 run match_cooperative_handling "
                + "set_relative_pose_from_current_object.py --ros-args "
                + f"-p robot_name:={robot} "
                + f"-p arms:={arms} "
                + f"-p world_frame:={WORLD_FRAME} "
                + "-p object_frame:=virtual_object/base_link "
                + "-p max_distance:=2.0"
            )
            self.append_log(
                f"[gui] Setting current object-relative TCP offsets for {robot}: {arms}"
            )
            self.start_process(
                self.process_key(robot, "set_current_offsets"),
                self.remote_command(robot, cmd),
            )

    def open_object_jog(self):
        dialog = ObjectJogDialog(self.ros_bridge, self.context.window)
        dialog.show()
        dialog.raise_()
        dialog.activateWindow()
        self._jog_dialog = dialog

    def open_demos(self):
        dialog = DemoDialog(self, self.context.window)
        dialog.show()
        dialog.raise_()
        dialog.activateWindow()
        self._demo_dialog = dialog

    def start_demo(
        self,
        demo_name,
        xy_amplitude,
        z_lift,
        yaw_amplitude_deg,
        linear_velocity,
        angular_velocity,
        repetitions,
    ):
        sides = self.selected_sides()
        if not sides:
            self.append_log("[gui] Refusing demo: no arm selected")
            return
        blocked = []
        for robot, side in self.context.robot_arm_pairs(sides=sides):
            status = self.context.arm_status(robot, side)
            if status != "armed":
                blocked.append(f"{robot}/{SIDES[side]}={status}")
        if blocked:
            self.append_log(
                "[gui] Refusing demo: press START MOTION first; " + ", ".join(blocked)
            )
            return
        object_host = self.object_host()
        process = self.context.window.processes.get(self.process_key(object_host, "demo"))
        if process is not None and process.state() != QtCore.QProcess.NotRunning:
            self.append_log("[gui] demo already running")
            return
        cmd = (
            self.remote_setup_prefix()
            + "exec ros2 run match_cooperative_handling virtual_object_demo_runner --ros-args "
            + f"-p robot_name:={object_host} "
            + f"-p world_frame:={WORLD_FRAME} "
            + f"-p demo_name:={demo_name} "
            + f"-p xy_amplitude:={xy_amplitude:.4f} "
            + f"-p z_lift:={z_lift:.4f} "
            + f"-p yaw_amplitude_deg:={yaw_amplitude_deg:.3f} "
            + f"-p linear_velocity:={linear_velocity:.4f} "
            + f"-p angular_velocity:={angular_velocity:.4f} "
            + f"-p repetitions:={int(repetitions)} "
            + "-p publish_rate_hz:=500.0"
        )
        self.append_log(
            "[gui] Starting demo. The demo only publishes virtual object twist commands."
        )
        self.start_process(
            self.process_key(object_host, "demo"),
            self.remote_command(object_host, cmd),
        )

    def stop_demo(self):
        for robot in ROBOTS:
            process = self.context.window.processes.get(self.process_key(robot, "demo"))
            if process is not None and process.state() != QtCore.QProcess.NotRunning:
                self.append_log(f"[gui] stopping demo on {robot}")
                process.terminate()
                if not process.waitForFinished(1000):
                    process.kill()
        self.ros_bridge.publish_object_twist([0.0] * 6)

    def start_tracking_log(self):
        sides = self.selected_sides()
        if not sides:
            self.append_log("[gui] Refusing tracking log: no arm selected")
            return
        arms = ",".join(sides)
        for robot in self.selected_robots():
            output_dir = os.path.join(
                self.context.window.remote_ws(),
                "src",
                "match_cooperative_handling",
                "logs",
                "tracking",
            )
            cmd = (
                self.remote_setup_prefix()
                + "exec ros2 run match_cooperative_handling log_cooperative_tracking.py --ros-args "
                + f"-p robot_name:={robot} "
                + f"-p arms:={arms} "
                + "-p duration:=300.0 "
                + "-p sample_rate_hz:=50.0 "
                + f"-p output_dir:={shlex.quote(output_dir)}"
            )
            self.append_log(f"[gui] Starting cooperative tracking logger for {robot}: {arms}")
            self.start_process(
                self.process_key(robot, "tracking_log"),
                self.remote_command(robot, cmd),
            )

    def stop_tracking_log(self):
        for robot in ROBOTS:
            process = self.context.window.processes.get(self.process_key(robot, "tracking_log"))
            if process is not None and process.state() != QtCore.QProcess.NotRunning:
                self.append_log(f"[gui] stopping tracking logger on {robot}")
                self.ros_bridge.publish_tracking_stop()
                if process.waitForFinished(2000):
                    continue
                process.terminate()
                if not process.waitForFinished(1000):
                    process.kill()

    def start_motion(self):
        sides = self.selected_sides()
        if not sides:
            self.append_log("[gui] Refusing start: no arm selected")
            return
        pairs = self.context.robot_arm_pairs(sides=sides)
        freedrive = [
            f"{robot}/{SIDES[side]}"
            for robot, side in pairs
            if self.context.window.freedrive_active.get((robot, side), False)
        ]
        if freedrive:
            self.append_log(
                "[gui] Refusing start: disable freedrive first for " + ", ".join(freedrive)
            )
            return
        self.context.ensure_ur_ready(
            sides=sides,
            robots=self.selected_robots(),
            on_success=lambda: self._start_motion_after_ready(pairs),
        )

    def _start_motion_after_ready(self, pairs):
        blocked = []
        for robot, side in pairs:
            status = self.context.arm_status(robot, side)
            if not (status == "ready" or status == "armed"):
                blocked.append(f"{robot}/{SIDES[side]}={status}")
            if not self.context.ur_reverse_ready(robot, side):
                blocked.append(f"{robot}/{SIDES[side]}=UR reverse missing")
        if blocked:
            self.append_log("[gui] Refusing start: " + ", ".join(blocked))
            return
        for robot, side in pairs:
            service = f"/{robot}/{SIDES[side]}/virtual_object_tcp_transform_node/start"
            self.context.ros_worker.call_trigger(service, f"start {robot}/{SIDES[side]}")

    def stop_motion(self):
        self.stop_demo()
        self.ros_bridge.publish_object_twist([0.0] * 6)
        robots = self.selected_robots() or ROBOTS
        sides = self.selected_sides() or ["r", "l"]
        for robot in robots:
            for side in sides:
                service = f"/{robot}/{SIDES[side]}/virtual_object_tcp_transform_node/stop"
                self.context.ros_worker.call_trigger(service, f"stop {robot}/{SIDES[side]}")

    def stop_motion_like_actions(self):
        self.stop_demo()
        self.stop_motion()
        self.stop_tracking_log()

    def on_hardware_start(self):
        self.stop_motion_like_actions()
        self.stop_object_nodes(start_after_cleanup=False)

    def on_shutdown(self):
        self.stop_object_nodes(start_after_cleanup=False)
        if self.ros_bridge is not None:
            self.ros_bridge.shutdown()
            self.ros_bridge.wait(1500)

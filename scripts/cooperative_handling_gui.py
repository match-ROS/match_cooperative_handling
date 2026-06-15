#!/usr/bin/env python3
"""Local PyQt GUI for the cooperative handling virtual object layer."""

import os
import shlex
import signal
import sys
import threading
import time
from functools import partial

from PyQt5 import QtCore, QtGui, QtWidgets

import rclpy
from controller_manager_msgs.srv import (
    ConfigureController,
    ListControllers,
    LoadController,
    SwitchController,
)
from geometry_msgs.msg import TwistStamped
from lifecycle_msgs.msg import State, TransitionEvent
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


WS = os.environ.get("WS", "/home/rosmatch/colcon_ws")
HARDWARE_SCRIPT = os.path.join(
    WS, "src", "match_mobile_robotics_jazzy", "start_mur620_hardware_logged.sh"
)
HARDWARE_LATEST_LOG = os.path.join(
    WS, "src", "match_mobile_robotics_jazzy", "logs", "hardware", "latest.log"
)
PACKAGE = "match_cooperative_handling"
SIDES = {"r": "UR10_r", "l": "UR10_l"}
FREEDRIVE_CONTROLLER = "freedrive_mode_controller"
FREEDRIVE_ENABLE_WAIT_SEC = 3.0
FREEDRIVE_ACTIVE_TRANSITION_WAIT_SEC = 4.0
FREEDRIVE_KEEPALIVE_HZ = 10.0
UR_REVERSE_READY_TEXT = "Robot connected to reverse interface. Ready to receive control commands."
UR_REVERSE_WAIT_SEC = 12.0
UR_READY_RETRY_LIMIT = 1
MOTION_CONTROLLERS = [
    "integrated_cartesian_admittance_controller",
    "forward_velocity_controller",
    "scaled_joint_trajectory_controller",
    "joint_trajectory_controller",
    "forward_position_controller",
    "forward_effort_controller",
    "force_mode_controller",
    "passthrough_trajectory_controller",
    "tool_contact_controller",
    FREEDRIVE_CONTROLLER,
]


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
    freedrive_status = QtCore.pyqtSignal(str, bool, str)

    def __init__(self, robot_name="mur620d"):
        super().__init__()
        self.robot_name = robot_name
        self._node = None
        self._object_twist_pub = None
        self._tracking_stop_pub = None
        self._status_subs = []
        self._previous_freedrive_controllers = {}
        self._freedrive_enable_pubs = {}
        self._freedrive_keepalive = {}
        self._freedrive_keepalive_last = {}
        self._ready = threading.Event()
        self._stop = threading.Event()
        self._lock = threading.Lock()

    def run(self):
        rclpy.init(args=None)
        self._node = rclpy.create_node("cooperative_handling_gui")
        self._object_twist_pub = self._node.create_publisher(
            TwistStamped, "/virtual_object/object_twist_cmd", 10
        )
        self._tracking_stop_pub = self._node.create_publisher(
            Bool, "/cooperative_tracking_logger/stop", 10
        )
        self._configure_status_subscriptions(self.robot_name)
        self._ready.set()
        self.log.emit("[ros] GUI ROS helper started")
        while rclpy.ok() and not self._stop.is_set():
            rclpy.spin_once(self._node, timeout_sec=0.05)
            self._publish_freedrive_keepalives()
        self.publish_object_twist([0.0] * 6)
        self._stop_all_freedrive_keepalives()
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

    def _duration_msg(self, seconds):
        duration = SwitchController.Request().timeout
        duration.sec = int(seconds)
        duration.nanosec = int((seconds - int(seconds)) * 1_000_000_000)
        return duration

    def _wait_for_future(self, future, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline:
            self.msleep(20)
        return future.done()

    def _controller_manager(self, robot_name, side):
        return f"/{robot_name}/{SIDES[side]}/controller_manager"

    def _list_controller_states(self, controller_manager):
        with self._lock:
            client = self._node.create_client(ListControllers, f"{controller_manager}/list_controllers")
        if not client.wait_for_service(timeout_sec=1.0):
            return None, f"service unavailable: {controller_manager}/list_controllers"
        future = client.call_async(ListControllers.Request())
        if not self._wait_for_future(future, 2.0):
            return None, f"timeout listing controllers at {controller_manager}"
        response = future.result()
        if response is None:
            return None, f"empty list_controllers response from {controller_manager}"
        return {controller.name: controller.state for controller in response.controller}, ""

    def _load_controller(self, controller_manager, controller_name):
        with self._lock:
            client = self._node.create_client(LoadController, f"{controller_manager}/load_controller")
        if not client.wait_for_service(timeout_sec=2.0):
            return False, f"service unavailable: {controller_manager}/load_controller"
        request = LoadController.Request()
        request.name = controller_name
        future = client.call_async(request)
        if not self._wait_for_future(future, 3.0):
            return False, f"timeout loading {controller_name}"
        response = future.result()
        if response is None or not response.ok:
            return False, f"failed loading {controller_name}"
        return True, f"loaded {controller_name}"

    def _configure_controller(self, controller_manager, controller_name):
        with self._lock:
            client = self._node.create_client(
                ConfigureController, f"{controller_manager}/configure_controller"
            )
        if not client.wait_for_service(timeout_sec=2.0):
            return False, f"service unavailable: {controller_manager}/configure_controller"
        request = ConfigureController.Request()
        request.name = controller_name
        future = client.call_async(request)
        if not self._wait_for_future(future, 3.0):
            return False, f"timeout configuring {controller_name}"
        response = future.result()
        if response is None or not response.ok:
            return False, f"failed configuring {controller_name}"
        return True, f"configured {controller_name}"

    def _ensure_controller_loaded(self, controller_manager, controller_name):
        states, error = self._list_controller_states(controller_manager)
        if states is None:
            return None, error
        if controller_name not in states:
            ok, message = self._load_controller(controller_manager, controller_name)
            self.log.emit(f"[ros] {controller_name}: {message}")
            if not ok:
                return None, message
            states, error = self._list_controller_states(controller_manager)
            if states is None:
                return None, error

        if states.get(controller_name) == "unconfigured":
            ok, message = self._configure_controller(controller_manager, controller_name)
            self.log.emit(f"[ros] {controller_name}: {message}")
            if not ok:
                return None, message
            states, error = self._list_controller_states(controller_manager)
            if states is None:
                return None, error
        return states, ""

    def _wait_for_controller_state(self, controller_manager, controller_name, target_state, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        last_state = "missing"
        last_error = ""
        while time.monotonic() < deadline:
            states, error = self._list_controller_states(controller_manager)
            if states is None:
                last_error = error
            else:
                last_state = states.get(controller_name, "missing")
                if last_state == target_state:
                    return True, f"{controller_name} is {target_state}"
            self.msleep(50)
        detail = last_error or f"last state was '{last_state}'"
        return False, f"timeout waiting for {controller_name} to become {target_state}: {detail}"

    def _freedrive_enable_topic(self, robot_name, side):
        return f"/{robot_name}/{SIDES[side]}/{FREEDRIVE_CONTROLLER}/enable_freedrive_mode"

    def _freedrive_transition_topic(self, robot_name, side):
        return f"/{robot_name}/{SIDES[side]}/{FREEDRIVE_CONTROLLER}/transition_event"

    def _get_freedrive_enable_publisher(self, robot_name, side):
        topic = self._freedrive_enable_topic(robot_name, side)
        with self._lock:
            publisher = self._freedrive_enable_pubs.get(topic)
            if publisher is None:
                publisher = self._node.create_publisher(Bool, topic, 10)
                self._freedrive_enable_pubs[topic] = publisher
        return topic, publisher

    def _create_freedrive_active_transition_waiter(self, robot_name, side):
        topic = self._freedrive_transition_topic(robot_name, side)
        event = threading.Event()
        details = {"message": "no transition_event received"}

        def callback(msg):
            goal_label = msg.goal_state.label
            goal_id = msg.goal_state.id
            transition_label = msg.transition.label
            details["message"] = (
                f"transition_event {transition_label}: "
                f"{msg.start_state.label}->{goal_label} ({goal_id})"
            )
            if goal_id == State.PRIMARY_STATE_ACTIVE or goal_label == "active":
                event.set()

        with self._lock:
            subscription = self._node.create_subscription(
                TransitionEvent,
                topic,
                callback,
                10,
            )

        def destroy():
            with self._lock:
                self._node.destroy_subscription(subscription)

        return event, details, destroy

    def _publish_freedrive_enable(self, robot_name, side, enabled):
        topic, publisher = self._get_freedrive_enable_publisher(robot_name, side)

        deadline = time.monotonic() + FREEDRIVE_ENABLE_WAIT_SEC
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            self.msleep(20)

        subscription_count = publisher.get_subscription_count()
        msg = Bool()
        msg.data = bool(enabled)
        publisher.publish(msg)

        if subscription_count == 0:
            return (
                False,
                f"Published {msg.data} on {topic}, but no subscriber was discovered "
                f"within {FREEDRIVE_ENABLE_WAIT_SEC:.1f}s",
            )
        return (
            True,
            f"Published {msg.data} once on {topic} (subscribers={subscription_count})",
        )

    def _set_freedrive_keepalive(self, robot_name, side, enabled):
        key = (robot_name, side)
        with self._lock:
            self._freedrive_keepalive[key] = bool(enabled)
            self._freedrive_keepalive_last[key] = 0.0

    def _publish_freedrive_keepalives(self):
        period = 1.0 / FREEDRIVE_KEEPALIVE_HZ
        now = time.monotonic()
        with self._lock:
            items = list(self._freedrive_keepalive.items())

        for (robot_name, side), enabled in items:
            if not enabled:
                continue
            last = self._freedrive_keepalive_last.get((robot_name, side), 0.0)
            if now - last < period:
                continue
            _topic, publisher = self._get_freedrive_enable_publisher(robot_name, side)
            msg = Bool()
            msg.data = True
            publisher.publish(msg)
            self._freedrive_keepalive_last[(robot_name, side)] = now

    def _stop_all_freedrive_keepalives(self):
        with self._lock:
            keys = list(self._freedrive_keepalive.keys())
        for robot_name, side in keys:
            self._set_freedrive_keepalive(robot_name, side, False)
            self._publish_freedrive_enable(robot_name, side, False)

    def _switch_controllers(self, controller_manager, activate, deactivate, label):
        with self._lock:
            client = self._node.create_client(SwitchController, f"{controller_manager}/switch_controller")
        if not client.wait_for_service(timeout_sec=2.0):
            return False, f"{label}: service unavailable: {controller_manager}/switch_controller"

        states, error = self._list_controller_states(controller_manager)
        if states is not None:
            activate = [name for name in activate if states.get(name) != "active"]
            deactivate = [name for name in deactivate if states.get(name) == "active"]
        if not activate and not deactivate:
            return True, f"{label}: controller state already correct"

        request = SwitchController.Request()
        request.activate_controllers = activate
        request.deactivate_controllers = deactivate
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.activate_asap = True
        request.timeout = self._duration_msg(5.0)
        self.log.emit(f"[ros] {label}: activate={activate}, deactivate={deactivate}")
        future = client.call_async(request)
        if not self._wait_for_future(future, 6.0):
            return False, f"{label}: timeout while switching controllers"
        response = future.result()
        if response is None or not response.ok:
            message = "" if response is None else response.message
            return False, f"{label}: switch failed: {message}"
        return True, f"{label}: switch ok"

    def switch_freedrive(self, robot_name, side, enable, fallback_controller):
        thread = threading.Thread(
            target=self._switch_freedrive_worker,
            args=(robot_name, side, enable, fallback_controller),
            daemon=True,
        )
        thread.start()

    def _switch_freedrive_worker(self, robot_name, side, enable, fallback_controller):
        if not self._ready.wait(timeout=1.0):
            self.log.emit(f"[ros] Cannot switch freedrive for {SIDES[side]}: ROS helper not ready")
            return
        controller_manager = self._controller_manager(robot_name, side)
        key = (robot_name, side)
        if enable:
            states, error = self._ensure_controller_loaded(controller_manager, FREEDRIVE_CONTROLLER)
            if states is None:
                self.log.emit(f"[ros] Freedrive {SIDES[side]}: {error}")
                self.freedrive_status.emit(side, False, error)
                return
            active_motion = [
                name
                for name in MOTION_CONTROLLERS
                if name != FREEDRIVE_CONTROLLER and states.get(name) == "active"
            ]
            if active_motion:
                self._previous_freedrive_controllers[key] = active_motion
            elif key not in self._previous_freedrive_controllers:
                self._previous_freedrive_controllers[key] = [fallback_controller]
            transition_event, transition_details, destroy_transition_sub = (
                self._create_freedrive_active_transition_waiter(robot_name, side)
            )
            try:
                ok, message = self._switch_controllers(
                    controller_manager,
                    activate=[FREEDRIVE_CONTROLLER],
                    deactivate=active_motion,
                    label=f"Freedrive ON {SIDES[side]}",
                )
                if ok:
                    saw_transition = transition_event.wait(
                        timeout=FREEDRIVE_ACTIVE_TRANSITION_WAIT_SEC
                    )
                    if saw_transition:
                        message += f"; {transition_details['message']}"
                    else:
                        state_ok, active_message = self._wait_for_controller_state(
                            controller_manager,
                            FREEDRIVE_CONTROLLER,
                            "active",
                            timeout_sec=0.5,
                        )
                        if state_ok:
                            message += (
                                "; no active transition_event observed, "
                                f"but {active_message}"
                            )
                        else:
                            ok = False
                            message += (
                                "; timeout waiting for active transition_event "
                                f"on {self._freedrive_transition_topic(robot_name, side)}; "
                                f"{transition_details['message']}; {active_message}"
                            )
                if ok:
                    publish_ok, publish_message = self._publish_freedrive_enable(
                        robot_name, side, True
                    )
                    message += f"; {publish_message}"
                    ok = publish_ok
                if ok:
                    self._set_freedrive_keepalive(robot_name, side, True)
                    message += f"; keepalive started at {FREEDRIVE_KEEPALIVE_HZ:.1f} Hz"
            finally:
                destroy_transition_sub()
            self.log.emit(f"[ros] {message}")
            self.freedrive_status.emit(side, ok, message)
            return

        restore = self._previous_freedrive_controllers.get(key) or [fallback_controller]
        restore = [name for name in restore if name and name != FREEDRIVE_CONTROLLER]
        self._set_freedrive_keepalive(robot_name, side, False)
        publish_ok, publish_message = self._publish_freedrive_enable(robot_name, side, False)
        self.log.emit(f"[ros] Freedrive OFF {SIDES[side]}: {publish_message}")
        ok, message = self._switch_controllers(
            controller_manager,
            activate=restore,
            deactivate=[FREEDRIVE_CONTROLLER],
            label=f"Freedrive OFF {SIDES[side]}",
        )
        if publish_ok:
            message += "; enable_freedrive_mode=false sent"
        self.log.emit(f"[ros] {message}")
        self.freedrive_status.emit(side, False if ok else True, message)

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

    def publish_tracking_stop(self):
        if self._tracking_stop_pub is None:
            return
        msg = Bool()
        msg.data = True
        for _ in range(5):
            self._tracking_stop_pub.publish(msg)
            self.msleep(20)


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


class DemoDialog(QtWidgets.QDialog):
    def __init__(self, main_window, parent=None):
        super().__init__(parent)
        self.main_window = main_window
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
        stop.clicked.connect(self.main_window.stop_demo)
        buttons.addWidget(start)
        buttons.addWidget(stop)
        layout.addLayout(buttons)

    def start_demo(self):
        self.main_window.start_demo(
            demo_name=self.demo_combo.currentData(),
            xy_amplitude=self.xy_amplitude.value(),
            z_lift=self.z_lift.value(),
            yaw_amplitude_deg=self.yaw_amplitude.value(),
            linear_velocity=self.linear_velocity.value(),
            angular_velocity=self.angular_velocity.value(),
            repetitions=self.repetitions.value(),
        )

    def closeEvent(self, event):
        self.main_window.stop_demo()
        super().closeEvent(event)


class CooperativeHandlingGui(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MuR Cooperative Handling")
        self.resize(1180, 780)
        self.processes = {}
        self.arm_status = {"r": "unknown", "l": "unknown"}
        self.ur_reverse_ready = {"r": False, "l": False}
        self.freedrive_active = {"r": False, "l": False}

        self.ros_worker = RosWorker("mur620")
        self.ros_worker.log.connect(self.append_log)
        self.ros_worker.status.connect(self.update_arm_status)
        self.ros_worker.freedrive_status.connect(self.update_freedrive_status)
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
            self._button("Ensure UR Ready", self.ensure_ur_ready),
            self._button("Start Object Nodes", self.start_object_nodes),
            self._button("Set From TCP", self.set_from_tcp),
            self._button("Home L", partial(self.move_home, "l")),
            self._button("Home R", partial(self.move_home, "r")),
            self._button("Open Object Jog", self.open_object_jog),
            self._button("Freedrive", self.toggle_freedrive),
            self._button("Stop Managed Processes", self.stop_managed_processes),
        ):
            actions.addWidget(button)
            if button.text() == "Freedrive":
                self.freedrive_button = button
                self.update_freedrive_button()

        tools = QtWidgets.QHBoxLayout()
        root.addLayout(tools)
        for button in (
            self._button("Open RViz", self.open_rviz),
            self._button("Demos", self.open_demos),
            self._button("Start Tracking Log", self.start_tracking_log),
            self._button("Stop Tracking Log", self.stop_tracking_log),
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
        self.arm_r.toggled.connect(lambda _checked: self.update_freedrive_button())
        self.arm_l = QtWidgets.QCheckBox("UR10_l")
        self.arm_l.setChecked(True)
        self.arm_l.toggled.connect(lambda _checked: self.update_freedrive_button())
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
        self.opt_zero_admittance = self._check("Zero admittance", False)
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
                self.opt_zero_admittance,
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
            self.opt_zero_admittance,
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

    def fallback_motion_controller(self):
        if self.opt_integrated.isChecked():
            return "integrated_cartesian_admittance_controller"
        return "forward_velocity_controller"

    def on_ros_name_changed(self):
        self.ros_worker.set_robot_name(self.robot_name())
        self.arm_status = {"r": "unknown", "l": "unknown"}
        self.ur_reverse_ready = {"r": False, "l": False}
        self.refresh_status_labels()

    def update_arm_status(self, side, status):
        self.arm_status[side] = status
        self.refresh_status_label(side)

    def set_ur_reverse_ready(self, side, ready, reason):
        if self.ur_reverse_ready.get(side) == ready:
            return
        self.ur_reverse_ready[side] = ready
        state = "ready" if ready else "not ready"
        self.append_log(f"[gui] {SIDES[side]} UR reverse interface {state}: {reason}")
        self.refresh_status_label(side)

    def refresh_status_label(self, side):
        gate_status = self.arm_status.get(side, "unknown")
        reverse_status = "UR reverse OK" if self.ur_reverse_ready.get(side, False) else "UR reverse missing"
        (self.status_r if side == "r" else self.status_l).setText(
            f"{gate_status} | {reverse_status}"
        )

    def refresh_status_labels(self):
        for side in SIDES:
            self.refresh_status_label(side)

    def update_freedrive_status(self, side, active, message):
        self.freedrive_active[side] = active
        self.append_log(f"[gui] {SIDES[side]} freedrive={'ON' if active else 'OFF'}: {message}")
        self.update_freedrive_button()

    def update_freedrive_button(self):
        if not hasattr(self, "freedrive_button"):
            return
        selected = self.selected_sides()
        selected_active = [self.freedrive_active.get(side, False) for side in selected]
        if selected and all(selected_active):
            self.freedrive_button.setText("Freedrive ON")
            self.freedrive_button.setStyleSheet(
                "QPushButton { background: #d69e2e; color: black; font-weight: bold; }"
            )
        elif any(self.freedrive_active.values()):
            active = ",".join(
                SIDES[side] for side, value in self.freedrive_active.items() if value
            )
            self.freedrive_button.setText(f"Freedrive {active}")
            self.freedrive_button.setStyleSheet(
                "QPushButton { background: #faf089; color: black; font-weight: bold; }"
            )
        else:
            self.freedrive_button.setText("Freedrive")
            self.freedrive_button.setStyleSheet("")

    def append_log(self, text):
        self.terminal.appendPlainText(text.rstrip())
        self.terminal.verticalScrollBar().setValue(self.terminal.verticalScrollBar().maximum())

    def start_process(self, name, command, env=None, on_finished=None):
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
        if on_finished is not None:
            process.finished.connect(on_finished)
        self.processes[name] = process
        self.append_log(f"[{name}] $ {command}")
        process.start("bash", ["-lc", command])

    def _read_process_output(self, tag, process):
        data = bytes(process.readAllStandardOutput()).decode(errors="replace")
        for line in data.splitlines():
            self.append_log(f"[{tag}] {line}")
            if tag == "hardware":
                self._observe_hardware_line(line)

    def _observe_hardware_line(self, line):
        if UR_REVERSE_READY_TEXT in line:
            for side, prefix in SIDES.items():
                if prefix in line:
                    self.set_ur_reverse_ready(side, True, "reverse interface connected")
            return

        if "UR SetMode goal was rejected" in line:
            for side, prefix in SIDES.items():
                if prefix in line:
                    self.set_ur_reverse_ready(side, False, "SetMode goal rejected")

    def _scan_latest_hardware_log_for_reverse_ready(self, sides):
        try:
            with open(HARDWARE_LATEST_LOG, "rb") as log_file:
                log_file.seek(0, os.SEEK_END)
                size = log_file.tell()
                log_file.seek(max(0, size - 512_000), os.SEEK_SET)
                text = log_file.read().decode(errors="replace")
        except OSError:
            return

        for line in text.splitlines():
            if UR_REVERSE_READY_TEXT not in line:
                continue
            for side in sides:
                if SIDES[side] in line:
                    self.set_ur_reverse_ready(
                        side, True, f"reverse interface seen in {HARDWARE_LATEST_LOG}"
                    )

    def start_hardware(self):
        self.stop_demo()
        self.ros_worker.publish_object_twist([0.0] * 6)
        self.stop_object_nodes(start_after_cleanup=False)
        profile = self.robot_profile()
        for side in self.selected_sides():
            self.ur_reverse_ready[side] = False
            self.refresh_status_label(side)
        args = [
            f"launch_ur_r:={'true' if self.arm_r.isChecked() else 'false'}",
            f"launch_ur_l:={'true' if self.arm_l.isChecked() else 'false'}",
            f"integrated_controller_enable_collision_avoidance:={'true' if self.opt_collision.isChecked() else 'false'}",
            f"integrated_controller_publish_collision_markers:={'true' if self.opt_markers.isChecked() else 'false'}",
            f"launch_moveit:={'true' if self.opt_moveit.isChecked() else 'false'}",
            "auto_switch_moveit_controllers:=true",
            "launch_moveit_rviz:=false",
        ]
        if self.opt_zero_admittance.isChecked():
            args.extend([
                "integrated_controller_admittance:=0.0 0.0 0.0 0.0 0.0 0.0",
                "integrated_controller_wrench_twist_gain:=0.0 0.0 0.0 0.0 0.0 0.0",
            ])
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

    def ensure_ur_ready(self, sides=None, on_success=None, retry_count=0):
        if isinstance(sides, bool):
            sides = None
        selected = list(sides) if sides is not None else self.selected_sides()
        if not selected:
            self.append_log("[gui] Refusing UR ready check: no arm selected")
            return False
        robot = self.robot_name()
        commands = []
        for side in selected:
            prefix = SIDES[side]
            commands.append(
                "ros2 run match_cooperative_handling ensure_ur_ready.py --ros-args "
                + f"-p arm_namespace:=/{robot}/{prefix} "
                + "-p wait_timeout:=30.0 "
                + "-p target_robot_mode:=7 "
                + "-p allow_stop_restart:=true"
            )
        command = setup_prefix() + " && ".join(commands)

        def retry(reason):
            if retry_count >= UR_READY_RETRY_LIMIT:
                self.append_log(
                    "[gui] UR ready check failed after retry. Not arming motion. "
                    f"Reason: {reason}"
                )
                return
            self.append_log(
                "[gui] UR ready check did not reach reverse-interface-ready; "
                f"retrying once. Reason: {reason}"
            )
            QtCore.QTimer.singleShot(
                1500,
                lambda: self.ensure_ur_ready(
                    sides=selected,
                    on_success=on_success,
                    retry_count=retry_count + 1,
                ),
            )

        def done(exit_code, _status):
            if exit_code == 0:
                self._wait_for_ur_reverse_ready(
                    selected,
                    on_success=on_success,
                    on_timeout=lambda missing: retry(
                        "missing reverse interface for "
                        + ", ".join(SIDES[side] for side in missing)
                    ),
                )
            else:
                retry("ensure_ur_ready script exited with failure")

        self.append_log(
            "[gui] Ensuring selected URs are running their External Control program: "
            + ", ".join(SIDES[side] for side in selected)
        )
        self.start_process("ensure_ur_ready", command, on_finished=done)
        return True

    def _wait_for_ur_reverse_ready(self, sides, on_success=None, on_timeout=None):
        deadline = time.monotonic() + UR_REVERSE_WAIT_SEC

        def poll():
            self._scan_latest_hardware_log_for_reverse_ready(sides)
            missing = [side for side in sides if not self.ur_reverse_ready.get(side, False)]
            if not missing:
                self.append_log(
                    "[gui] UR reverse interface ready for: "
                    + ", ".join(SIDES[side] for side in sides)
                )
                if on_success is not None:
                    QtCore.QTimer.singleShot(200, on_success)
                return
            if time.monotonic() >= deadline:
                self.append_log(
                    "[gui] UR ready check timed out waiting for reverse interface: "
                    + ", ".join(SIDES[side] for side in missing)
                )
                if on_timeout is not None:
                    on_timeout(missing)
                return
            QtCore.QTimer.singleShot(250, poll)

        poll()

    def start_object_nodes(self):
        self.stop_demo()
        self.ros_worker.publish_object_twist([0.0] * 6)
        self.stop_object_nodes(start_after_cleanup=True)

    def stop_object_nodes(self, start_after_cleanup=False):
        cleanup_process = self.processes.get("object_cleanup")
        if cleanup_process is not None and cleanup_process.state() != QtCore.QProcess.NotRunning:
            self.append_log("[gui] terminating previous object_cleanup")
            cleanup_process.terminate()
            if not cleanup_process.waitForFinished(1000):
                cleanup_process.kill()

        for name in ("object_state", "object_transform_r", "object_transform_l"):
            process = self.processes.get(name)
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
        robot = self.robot_name()
        state_cmd = (
            setup_prefix()
            + "exec ros2 run match_cooperative_handling virtual_object_state_node --ros-args "
            + f"-p world_frame:={robot}/base_link "
            + "-p rate:=500.0"
        )
        self.start_process("object_state", state_cmd)
        for side in self.selected_sides():
            prefix = SIDES[side]
            transform_cmd = (
                setup_prefix()
                + "exec ros2 run match_cooperative_handling virtual_object_tcp_transform_node --ros-args "
                + f"-r __ns:=/{robot}/{prefix} "
                + f"-p robot_name:={robot} "
                + f"-p arm:={side} "
                + "-p rate:=500.0"
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

    def toggle_freedrive(self):
        sides = self.selected_sides()
        if not sides:
            self.append_log("[gui] Refusing freedrive: no arm selected")
            return
        enable = not all(self.freedrive_active.get(side, False) for side in sides)
        fallback = self.fallback_motion_controller()
        if enable:
            self.append_log(
                "[gui] Enabling freedrive: stopping demos, object twists, and motion gates first"
            )
            self.stop_demo()
            self.ros_worker.publish_object_twist([0.0] * 6)
            for side in sides:
                service = f"/{self.robot_name()}/{SIDES[side]}/virtual_object_tcp_transform_node/stop"
                self.ros_worker.call_trigger(service, f"disarm before freedrive {SIDES[side]}")
                self.ros_worker.switch_freedrive(self.robot_name(), side, True, fallback)
            return

        self.append_log("[gui] Disabling freedrive and restoring previous motion controllers")
        for side in sides:
            self.ros_worker.switch_freedrive(self.robot_name(), side, False, fallback)

    def open_demos(self):
        dialog = DemoDialog(self, self)
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
        for side in sides:
            status = self.arm_status.get(side, "unknown")
            if status != "armed":
                blocked.append(f"{SIDES[side]}={status}")
        if blocked:
            self.append_log(
                "[gui] Refusing demo: press START MOTION first; " + ", ".join(blocked)
            )
            return
        process = self.processes.get("demo")
        if process is not None and process.state() != QtCore.QProcess.NotRunning:
            self.append_log("[gui] demo already running")
            return
        robot = self.robot_name()
        cmd = (
            setup_prefix()
            + "exec ros2 run match_cooperative_handling virtual_object_demo_runner --ros-args "
            + f"-p robot_name:={robot} "
            + f"-p world_frame:={robot}/base_link "
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
        self.start_process("demo", cmd)

    def stop_demo(self):
        process = self.processes.get("demo")
        if process is not None and process.state() != QtCore.QProcess.NotRunning:
            self.append_log("[gui] stopping demo")
            process.terminate()
            if not process.waitForFinished(1000):
                process.kill()
        self.ros_worker.publish_object_twist([0.0] * 6)

    def start_tracking_log(self):
        sides = self.selected_sides()
        if not sides:
            self.append_log("[gui] Refusing tracking log: no arm selected")
            return
        robot = self.robot_name()
        arms = ",".join(sides)
        output_dir = os.path.join(WS, "src", "match_cooperative_handling", "logs", "tracking")
        cmd = (
            setup_prefix()
            + "exec ros2 run match_cooperative_handling log_cooperative_tracking.py --ros-args "
            + f"-p robot_name:={robot} "
            + f"-p arms:={arms} "
            + "-p duration:=300.0 "
            + "-p sample_rate_hz:=50.0 "
            + f"-p output_dir:={shlex.quote(output_dir)}"
        )
        self.append_log(f"[gui] Starting cooperative tracking logger for: {arms}")
        self.start_process("tracking_log", cmd)

    def stop_tracking_log(self):
        process = self.processes.get("tracking_log")
        if process is not None and process.state() != QtCore.QProcess.NotRunning:
            self.append_log("[gui] stopping tracking logger")
            self.ros_worker.publish_tracking_stop()
            if process.waitForFinished(2000):
                return
            process.terminate()
            if not process.waitForFinished(1000):
                process.kill()

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
        if self.freedrive_active.get(side, False):
            self.append_log(f"[gui] Home {prefix}: disabling freedrive first")
            self.ros_worker.switch_freedrive(
                self.robot_name(), side, False, self.fallback_motion_controller()
            )
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
        freedrive = [SIDES[side] for side in sides if self.freedrive_active.get(side, False)]
        if freedrive:
            self.append_log(
                "[gui] Refusing start: disable freedrive first for " + ", ".join(freedrive)
            )
            return
        self.ensure_ur_ready(sides=sides, on_success=lambda: self._start_motion_after_ready(sides))

    def _start_motion_after_ready(self, sides):
        blocked = []
        for side in sides:
            status = self.arm_status.get(side, "unknown")
            if not (status == "ready" or status == "armed"):
                blocked.append(f"{SIDES[side]}={status}")
            if not self.ur_reverse_ready.get(side, False):
                blocked.append(f"{SIDES[side]}=UR reverse missing")
        if blocked:
            self.append_log("[gui] Refusing start: " + ", ".join(blocked))
            return
        for side in sides:
            service = f"/{self.robot_name()}/{SIDES[side]}/virtual_object_tcp_transform_node/start"
            self.ros_worker.call_trigger(service, f"start {SIDES[side]}")

    def stop_motion(self):
        self.stop_demo()
        self.ros_worker.publish_object_twist([0.0] * 6)
        for side in self.selected_sides() or ["r", "l"]:
            service = f"/{self.robot_name()}/{SIDES[side]}/virtual_object_tcp_transform_node/stop"
            self.ros_worker.call_trigger(service, f"stop {SIDES[side]}")

    def stop_managed_processes(self):
        self.stop_tracking_log()
        self.stop_object_nodes(start_after_cleanup=False)
        for name, process in list(self.processes.items()):
            if name == "object_cleanup":
                continue
            if process.state() == QtCore.QProcess.NotRunning:
                continue
            self.append_log(f"[gui] terminating {name}")
            process.terminate()
            if not process.waitForFinished(1500):
                process.kill()

    def closeEvent(self, event):
        self.stop_demo()
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

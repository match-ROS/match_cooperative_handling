#!/usr/bin/env python3
"""Ensure a UR driver is in running external-control mode."""

import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger
from ur_dashboard_msgs.action import SetMode
from ur_dashboard_msgs.srv import IsProgramRunning


def parse_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


class EnsureURReady(Node):
    def __init__(self):
        super().__init__("ensure_ur_ready")
        self.declare_parameter("arm_namespace", "")
        self.declare_parameter("wait_timeout", 30.0)
        self.declare_parameter("target_robot_mode", 7)
        self.declare_parameter("allow_stop_restart", True)

        self.arm_namespace = self.get_parameter("arm_namespace").value.strip("/")
        if not self.arm_namespace:
            raise RuntimeError("arm_namespace must not be empty")
        self.wait_timeout = float(self.get_parameter("wait_timeout").value)
        self.service_timeout = max(0.5, min(self.wait_timeout, 5.0))
        self.target_robot_mode = int(self.get_parameter("target_robot_mode").value)
        self.allow_stop_restart = parse_bool(
            self.get_parameter("allow_stop_restart").value
        )

        self.program_running_client = self.create_client(
            IsProgramRunning,
            f"/{self.arm_namespace}/dashboard_client/program_running",
        )
        self.play_client = self.create_client(
            Trigger,
            f"/{self.arm_namespace}/dashboard_client/play",
        )
        self.stop_client = self.create_client(
            Trigger,
            f"/{self.arm_namespace}/dashboard_client/stop",
        )
        self.set_mode_client = ActionClient(
            self,
            SetMode,
            f"/{self.arm_namespace}/ur_robot_state_helper/set_mode",
        )
        self.get_logger().info(f"Ensuring UR ready: /{self.arm_namespace}")

    def _wait_for_future(self, future, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.05)
        return future.done()

    def program_running(self):
        if not self.program_running_client.wait_for_service(timeout_sec=self.service_timeout):
            self.get_logger().error("program_running service is unavailable")
            return None
        future = self.program_running_client.call_async(IsProgramRunning.Request())
        if not self._wait_for_future(future, self.service_timeout):
            self.get_logger().error("program_running service timed out")
            return None
        response = future.result()
        if response is None or not response.success:
            answer = "" if response is None else response.answer
            self.get_logger().warn(f"program_running query failed: {answer}")
            return None
        return bool(response.program_running)

    def wait_until_running(self, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            running = self.program_running()
            if running:
                return True
            time.sleep(0.2)
        return False

    def call_trigger(self, client, label, timeout=None):
        timeout = self.service_timeout if timeout is None else timeout
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(f"{label} service is unavailable")
            return False
        future = client.call_async(Trigger.Request())
        if not self._wait_for_future(future, timeout):
            self.get_logger().error(f"{label} service timed out")
            return False
        response = future.result()
        if response is None or not response.success:
            message = "" if response is None else response.message
            self.get_logger().error(f"{label} failed: {message}")
            return False
        self.get_logger().info(f"{label} succeeded: {response.message}")
        return True

    def set_mode(self, stop_program, play_program, timeout=None):
        timeout = max(self.service_timeout, self.wait_timeout)
        if not self.set_mode_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error("SetMode action is unavailable")
            return False
        goal = SetMode.Goal()
        goal.target_robot_mode = self.target_robot_mode
        goal.stop_program = bool(stop_program)
        goal.play_program = bool(play_program)
        self.get_logger().info(
            "Sending SetMode: "
            f"target_robot_mode={goal.target_robot_mode}, "
            f"stop_program={goal.stop_program}, play_program={goal.play_program}"
        )
        future = self.set_mode_client.send_goal_async(goal)
        if not self._wait_for_future(future, timeout):
            self.get_logger().error("SetMode goal request timed out")
            return False
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("SetMode goal was rejected")
            return False
        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, timeout):
            self.get_logger().error("SetMode result timed out")
            return False
        result = result_future.result().result
        if result.success or "Reached target robot mode" in result.message:
            self.get_logger().info(f"SetMode accepted: {result.message}")
            return True
        self.get_logger().error(f"SetMode failed: {result.message}")
        return False

    def run(self):
        running = self.program_running()
        if running:
            self.get_logger().info("UR program is already running.")
            return 0

        self.get_logger().warn("UR program is not running; trying Dashboard play.")
        if self.call_trigger(self.play_client, "Dashboard play"):
            if self.wait_until_running(self.service_timeout):
                self.get_logger().info("UR program is running after Dashboard play.")
                return 0

        self.get_logger().warn("Trying SetMode without stop_program.")
        if self.set_mode(stop_program=False, play_program=True):
            if self.wait_until_running(max(self.service_timeout, min(self.wait_timeout, 8.0))):
                self.get_logger().info("UR program is running after SetMode play.")
                return 0

        if self.allow_stop_restart:
            self.get_logger().warn("Trying stop+restart fallback.")
            self.call_trigger(self.stop_client, "Dashboard stop")
            if self.set_mode(stop_program=True, play_program=True):
                if self.wait_until_running(max(self.service_timeout, min(self.wait_timeout, 10.0))):
                    self.get_logger().info("UR program is running after stop+restart.")
                    return 0

        self.get_logger().error("UR program is still not running.")
        return 1


def main():
    rclpy.init()
    node = None
    try:
        node = EnsureURReady()
        return node.run()
    except Exception as exc:  # noqa: BLE001
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(str(exc), file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Small virtual-object demo motions for cooperative handling."""

import math
import threading
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node


@dataclass
class Segment:
    label: str
    axis: int
    distance: float
    max_velocity: float


class VirtualObjectDemoRunner(Node):
    def __init__(self):
        super().__init__("virtual_object_demo_runner")
        self.robot_name = self.declare_parameter("robot_name", "mur620").value
        self.demo_name = self.declare_parameter("demo_name", "safe_wiggle").value
        self.world_frame = self.declare_parameter("world_frame", f"{self.robot_name}/base_link").value
        self.twist_topic = self.declare_parameter(
            "twist_topic", "/virtual_object/object_twist_cmd"
        ).value
        self.object_pose_topic = self.declare_parameter(
            "object_pose_topic", "/virtual_object/object_pose"
        ).value
        self.xy_amplitude = float(self.declare_parameter("xy_amplitude", 0.05).value)
        self.z_lift = float(self.declare_parameter("z_lift", 0.05).value)
        self.yaw_amplitude_deg = float(
            self.declare_parameter("yaw_amplitude_deg", 5.0).value
        )
        self.linear_velocity = float(self.declare_parameter("linear_velocity", 0.02).value)
        self.angular_velocity = float(self.declare_parameter("angular_velocity", 0.10).value)
        self.repetitions = int(self.declare_parameter("repetitions", 1).value)
        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 50.0).value)
        self.pose_timeout = float(self.declare_parameter("pose_timeout", 3.0).value)
        self.min_segment_duration = float(
            self.declare_parameter("min_segment_duration", 1.0).value
        )

        self._pose_event = threading.Event()
        self._last_pose_time = None
        self._stop_requested = False

        self.twist_pub = self.create_publisher(TwistStamped, self.twist_topic, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, self.object_pose_topic, self._pose_cb, 10
        )

    def _pose_cb(self, _msg):
        self._last_pose_time = self.get_clock().now()
        self._pose_event.set()

    def wait_for_object_pose(self):
        self.get_logger().info(
            f"Waiting for virtual object pose on {self.object_pose_topic}"
        )
        end_time = self.get_clock().now().nanoseconds + int(self.pose_timeout * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._pose_event.is_set():
                self.get_logger().info("Virtual object pose is available")
                return True
        self.get_logger().error("Timed out waiting for virtual object pose")
        return False

    def publish_twist(self, values):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.world_frame
        msg.twist.linear.x = float(values[0])
        msg.twist.linear.y = float(values[1])
        msg.twist.linear.z = float(values[2])
        msg.twist.angular.x = float(values[3])
        msg.twist.angular.y = float(values[4])
        msg.twist.angular.z = float(values[5])
        self.twist_pub.publish(msg)

    def stop(self):
        self._stop_requested = True
        self.publish_twist([0.0] * 6)

    def _run_segment(self, segment):
        distance = float(segment.distance)
        max_velocity = max(abs(float(segment.max_velocity)), 1e-6)
        if abs(distance) < 1e-9:
            return True
        duration = max(self.min_segment_duration, 1.875 * abs(distance) / max_velocity)
        period = 1.0 / max(self.publish_rate_hz, 1.0)
        start_time = self.get_clock().now()
        self.get_logger().info(
            f"Segment {segment.label}: distance={distance:.4f}, duration={duration:.2f}s"
        )
        while rclpy.ok() and not self._stop_requested:
            rclpy.spin_once(self, timeout_sec=0.0)
            elapsed = (self.get_clock().now() - start_time).nanoseconds * 1e-9
            if elapsed >= duration:
                break
            u = max(0.0, min(1.0, elapsed / duration))
            velocity_scale = 30.0 * u * u - 60.0 * u * u * u + 30.0 * u * u * u * u
            velocity = distance / duration * velocity_scale
            values = [0.0] * 6
            values[segment.axis] = velocity
            self.publish_twist(values)
            time.sleep(period)
        self.publish_twist([0.0] * 6)
        return not self._stop_requested

    def _safe_wiggle_segments(self):
        yaw = math.radians(self.yaw_amplitude_deg)
        return [
            Segment("lift +Z", 2, self.z_lift, self.linear_velocity),
            Segment("X+", 0, self.xy_amplitude, self.linear_velocity),
            Segment("X-", 0, -2.0 * self.xy_amplitude, self.linear_velocity),
            Segment("X center", 0, self.xy_amplitude, self.linear_velocity),
            Segment("Y+", 1, self.xy_amplitude, self.linear_velocity),
            Segment("Y-", 1, -2.0 * self.xy_amplitude, self.linear_velocity),
            Segment("Y center", 1, self.xy_amplitude, self.linear_velocity),
            Segment("Yaw+", 5, yaw, self.angular_velocity),
            Segment("Yaw-", 5, -2.0 * yaw, self.angular_velocity),
            Segment("Yaw center", 5, yaw, self.angular_velocity),
            Segment("lower -Z", 2, -self.z_lift, self.linear_velocity),
        ]

    def run(self):
        if self.demo_name != "safe_wiggle":
            self.get_logger().error(f"Unsupported demo_name '{self.demo_name}'")
            return 2
        if self.linear_velocity <= 0.0 or self.angular_velocity <= 0.0:
            self.get_logger().error("Velocity limits must be positive")
            return 2
        if not self.wait_for_object_pose():
            self.stop()
            return 1
        self.get_logger().info(
            "Starting Safe Wiggle demo. This node only publishes object twist commands; "
            "motion gates must already be armed by the operator."
        )
        try:
            for rep in range(max(1, self.repetitions)):
                self.get_logger().info(f"Safe Wiggle repetition {rep + 1}/{self.repetitions}")
                for segment in self._safe_wiggle_segments():
                    if not self._run_segment(segment):
                        self.get_logger().warn("Demo stopped before completion")
                        return 130
            self.get_logger().info("Safe Wiggle demo complete")
            return 0
        finally:
            self.stop()


def main():
    rclpy.init()
    node = VirtualObjectDemoRunner()
    try:
        exit_code = node.run()
    except KeyboardInterrupt:
        node.stop()
        exit_code = 130
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()

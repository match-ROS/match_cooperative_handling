#!/usr/bin/env python3
"""Initialize a virtual object from any selected MuR TCPs in a shared map."""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from match_cooperative_handling.object_pose_math import (
    average_pose, offset_pose, parse_tcp_pairs, relative_pose,
)


def pose_msg(frame_id, stamp, transform):
    position, orientation = transform
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = position
    msg.pose.orientation.x, msg.pose.orientation.y = orientation[:2]
    msg.pose.orientation.z, msg.pose.orientation.w = orientation[2:]
    return msg


class SetVirtualObjectFromTcps(Node):
    def __init__(self):
        super().__init__("set_virtual_object_from_tcps")
        self.declare_parameter("tcp_pairs", "")
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("offset_frame", "object")
        for axis in ("x", "y", "z"):
            self.declare_parameter(f"offset_{axis}", 0.0)
        for axis in ("roll", "pitch", "yaw"):
            self.declare_parameter(f"offset_{axis}_deg", 0.0)
        self.declare_parameter("wait_timeout", 5.0)
        self.declare_parameter("publish_duration", 1.0)
        self.declare_parameter("publish_rate_hz", 20.0)

        self.pairs = parse_tcp_pairs(str(self.get_parameter("tcp_pairs").value))
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.offset_frame = str(self.get_parameter("offset_frame").value)
        self.offset_xyz = tuple(
            float(self.get_parameter(f"offset_{axis}").value) for axis in ("x", "y", "z")
        )
        self.offset_rpy_deg = tuple(
            float(self.get_parameter(f"offset_{axis}_deg").value)
            for axis in ("roll", "pitch", "yaw")
        )
        if not all(math.isfinite(value) for value in (*self.offset_xyz, *self.offset_rpy_deg)):
            raise ValueError("Offsets must be finite")
        if self.offset_frame not in ("object", "world"):
            raise ValueError("offset_frame must be 'object' or 'world'")
        self.wait_timeout = float(self.get_parameter("wait_timeout").value)
        self.publish_duration = float(self.get_parameter("publish_duration").value)
        self.publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))

        qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.object_pub = self.create_publisher(PoseStamped, "/virtual_object/set_pose", qos)
        self.relative_pubs = {
            pair: self.create_publisher(PoseStamped, self.relative_topic(pair), qos)
            for pair in self.pairs
        }
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    @staticmethod
    def tcp_frame(pair):
        robot, side = pair
        return f"{robot}/UR10_{side}/tool0"

    @staticmethod
    def relative_topic(pair):
        robot, side = pair
        return (
            f"/{robot}/UR10_{side}/virtual_object_tcp_transform_node/"
            "relative_object_to_tcp_pose"
        )

    def lookup_tcp(self, pair):
        frame = self.tcp_frame(pair)
        deadline = time.monotonic() + self.wait_timeout
        last_error = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                stamped = self.tf_buffer.lookup_transform(self.world_frame, frame, Time())
                position = stamped.transform.translation
                orientation = stamped.transform.rotation
                result = (
                    (position.x, position.y, position.z),
                    (orientation.x, orientation.y, orientation.z, orientation.w),
                )
                if not all(math.isfinite(value) for group in result for value in group):
                    raise RuntimeError(f"Invalid TF values for {frame}")
                return result
            except TransformException as exc:
                last_error = exc
        raise RuntimeError(f"Timed out waiting for TF {self.world_frame} -> {frame}: {last_error}")

    def run(self):
        # Receive existing TF subscriptions before requesting the first transform.
        warmup_deadline = time.monotonic() + 0.25
        while rclpy.ok() and time.monotonic() < warmup_deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

        tcp_poses = {pair: self.lookup_tcp(pair) for pair in self.pairs}
        center = average_pose(list(tcp_poses.values()))
        object_pose = offset_pose(center, self.offset_xyz, self.offset_rpy_deg, self.offset_frame)
        relatives = {
            pair: relative_pose(object_pose, tcp_pose)
            for pair, tcp_pose in tcp_poses.items()
        }

        end_time = time.monotonic() + self.publish_duration
        period = 1.0 / self.publish_rate_hz
        while rclpy.ok() and time.monotonic() < end_time:
            stamp = self.get_clock().now().to_msg()
            self.object_pub.publish(pose_msg(self.world_frame, stamp, object_pose))
            for pair, transform in relatives.items():
                self.relative_pubs[pair].publish(
                    pose_msg("virtual_object/base_link", stamp, transform)
                )
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

        self.get_logger().info(
            "Set virtual object from %s in %s; offset %s xyz=%s rpy_deg=%s"
            % (self.pairs, self.world_frame, self.offset_frame,
               self.offset_xyz, self.offset_rpy_deg)
        )


def main():
    rclpy.init()
    node = SetVirtualObjectFromTcps()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

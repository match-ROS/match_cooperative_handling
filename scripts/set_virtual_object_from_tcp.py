#!/usr/bin/env python3
"""Initialize the virtual object from a TCP pose plus a local TCP offset."""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def normalize_quat(q):
    norm = math.sqrt(sum(value * value for value in q))
    if norm <= 1.0e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(value / norm for value in q)


def quat_multiply_raw(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_multiply(a, b):
    return normalize_quat(quat_multiply_raw(a, b))


def quat_inverse(q):
    x, y, z, w = normalize_quat(q)
    return (-x, -y, -z, w)


def rotate_vector(q, v):
    qv = (v[0], v[1], v[2], 0.0)
    qr = quat_multiply_raw(quat_multiply_raw(normalize_quat(q), qv), quat_inverse(q))
    return (qr[0], qr[1], qr[2])


def rpy_to_quat(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return normalize_quat(
        (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )
    )


def transform_multiply(a, b):
    ap, aq = a
    bp, bq = b
    rbp = rotate_vector(aq, bp)
    return (
        (ap[0] + rbp[0], ap[1] + rbp[1], ap[2] + rbp[2]),
        quat_multiply(aq, bq),
    )


def transform_inverse(transform):
    p, q = transform
    qi = quat_inverse(q)
    rp = rotate_vector(qi, (-p[0], -p[1], -p[2]))
    return (rp, qi)


def pose_msg(frame_id, stamp, transform):
    p, q = transform
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.pose.position.x = p[0]
    msg.pose.position.y = p[1]
    msg.pose.position.z = p[2]
    msg.pose.orientation.x = q[0]
    msg.pose.orientation.y = q[1]
    msg.pose.orientation.z = q[2]
    msg.pose.orientation.w = q[3]
    return msg


class SetVirtualObjectFromTcp(Node):
    def __init__(self):
        super().__init__("set_virtual_object_from_tcp")
        self.declare_parameter("robot_name", "mur620")
        self.declare_parameter("arm", "r")
        self.declare_parameter("world_frame", "")
        self.declare_parameter("tcp_frame", "")
        self.declare_parameter("tcp_object_xyz", [0.10, 0.0, 0.0])
        self.declare_parameter("tcp_object_rpy", [0.0, 0.0, 0.0])
        self.declare_parameter("set_pose_topic", "/virtual_object/set_pose")
        self.declare_parameter("relative_pose_topic", "")
        self.declare_parameter("wait_timeout", 5.0)
        self.declare_parameter("publish_duration", 1.0)
        self.declare_parameter("publish_rate_hz", 20.0)

        self.robot_name = str(self.get_parameter("robot_name").value)
        self.arm = str(self.get_parameter("arm").value)
        self.arm_name = f"UR10_{self.arm}"
        world_default = f"{self.robot_name}/base_link"
        tcp_default = f"{self.robot_name}/{self.arm_name}/tool0"
        relative_default = (
            f"/{self.robot_name}/{self.arm_name}/virtual_object_tcp_transform_node/"
            "relative_object_to_tcp_pose"
        )
        self.world_frame = str(self.get_parameter("world_frame").value) or world_default
        self.tcp_frame = str(self.get_parameter("tcp_frame").value) or tcp_default
        self.set_pose_topic = str(self.get_parameter("set_pose_topic").value)
        self.relative_pose_topic = (
            str(self.get_parameter("relative_pose_topic").value) or relative_default
        )
        self.tcp_object_xyz = tuple(float(v) for v in self.get_parameter("tcp_object_xyz").value)
        self.tcp_object_rpy = tuple(float(v) for v in self.get_parameter("tcp_object_rpy").value)
        self.wait_timeout = float(self.get_parameter("wait_timeout").value)
        self.publish_duration = float(self.get_parameter("publish_duration").value)
        self.publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.object_pose_pub = self.create_publisher(PoseStamped, self.set_pose_topic, qos)
        self.relative_pose_pub = self.create_publisher(PoseStamped, self.relative_pose_topic, qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def lookup_tcp(self):
        warmup_deadline = time.monotonic() + 0.25
        while rclpy.ok() and time.monotonic() < warmup_deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

        deadline = time.monotonic() + self.wait_timeout
        last_error = None
        latest_time = Time(seconds=0, nanoseconds=0)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.world_frame,
                    self.tcp_frame,
                    latest_time,
                )
                p = transform.transform.translation
                q = transform.transform.rotation
                return ((p.x, p.y, p.z), normalize_quat((q.x, q.y, q.z, q.w)))
            except TransformException as exc:
                last_error = exc
        raise RuntimeError(
            f"Timed out waiting for TF {self.world_frame} -> {self.tcp_frame}: {last_error}"
        )

    def run(self):
        world_from_tcp = self.lookup_tcp()
        tcp_from_object = (
            self.tcp_object_xyz,
            rpy_to_quat(*self.tcp_object_rpy),
        )
        world_from_object = transform_multiply(world_from_tcp, tcp_from_object)
        object_from_tcp = transform_inverse(tcp_from_object)

        stamp = self.get_clock().now().to_msg()
        object_msg = pose_msg(self.world_frame, stamp, world_from_object)
        relative_msg = pose_msg("virtual_object/base_link", stamp, object_from_tcp)

        end_time = time.monotonic() + self.publish_duration
        period = 1.0 / self.publish_rate_hz
        while rclpy.ok() and time.monotonic() < end_time:
            stamp = self.get_clock().now().to_msg()
            object_msg.header.stamp = stamp
            relative_msg.header.stamp = stamp
            self.object_pose_pub.publish(object_msg)
            self.relative_pose_pub.publish(relative_msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

        self.get_logger().info(
            "Set virtual object from %s with tcp_object_xyz=%s. Object pose topic=%s, "
            "relative pose topic=%s"
            % (self.tcp_frame, self.tcp_object_xyz, self.set_pose_topic, self.relative_pose_topic)
        )


def main():
    rclpy.init()
    node = SetVirtualObjectFromTcp()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

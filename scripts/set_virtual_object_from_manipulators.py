#!/usr/bin/env python3
"""Initialize the virtual object at the average pose of selected TCP frames."""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


SIDES = {"l": "UR10_l", "r": "UR10_r"}


def normalize_quat(q):
    norm = math.sqrt(sum(value * value for value in q))
    if norm <= 1.0e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(value / norm for value in q)


def quat_dot(a, b):
    return sum(x * y for x, y in zip(a, b))


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


def parse_arms(value):
    arms = []
    for raw_part in str(value).replace(";", ",").split(","):
        part = raw_part.strip()
        if not part:
            continue
        if part.startswith("UR10_"):
            part = part[-1]
        if part not in SIDES:
            raise ValueError(f"Unsupported arm '{raw_part}'. Use l,r or UR10_l,UR10_r.")
        if part not in arms:
            arms.append(part)
    return arms


def average_pose(transforms):
    count = float(len(transforms))
    position = tuple(sum(transform[0][idx] for transform in transforms) / count for idx in range(3))

    reference = transforms[0][1]
    quat_sum = [0.0, 0.0, 0.0, 0.0]
    for _, quat in transforms:
        q = normalize_quat(quat)
        if quat_dot(reference, q) < 0.0:
            q = tuple(-value for value in q)
        for idx, value in enumerate(q):
            quat_sum[idx] += value
    return (position, normalize_quat(quat_sum))


class SetVirtualObjectFromManipulators(Node):
    def __init__(self):
        super().__init__("set_virtual_object_from_manipulators")
        self.declare_parameter("robot_name", "mur620")
        self.declare_parameter("arms", "l,r")
        self.declare_parameter("world_frame", "")
        self.declare_parameter("set_pose_topic", "/virtual_object/set_pose")
        self.declare_parameter("relative_pose_topic_template", "")
        self.declare_parameter("wait_timeout", 5.0)
        self.declare_parameter("publish_duration", 1.0)
        self.declare_parameter("publish_rate_hz", 20.0)

        self.robot_name = str(self.get_parameter("robot_name").value)
        self.arms = parse_arms(self.get_parameter("arms").value)
        self.world_frame = (
            str(self.get_parameter("world_frame").value) or f"{self.robot_name}/base_link"
        )
        self.set_pose_topic = str(self.get_parameter("set_pose_topic").value)
        self.relative_pose_topic_template = str(
            self.get_parameter("relative_pose_topic_template").value
        )
        self.wait_timeout = float(self.get_parameter("wait_timeout").value)
        self.publish_duration = float(self.get_parameter("publish_duration").value)
        self.publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))

        if len(self.arms) < 2:
            raise RuntimeError("At least two manipulators are required to set the object center.")

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.object_pose_pub = self.create_publisher(PoseStamped, self.set_pose_topic, qos)
        self.relative_pose_pubs = {
            arm: self.create_publisher(PoseStamped, self.relative_pose_topic(arm), qos)
            for arm in self.arms
        }
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def tcp_frame(self, arm):
        return f"{self.robot_name}/{SIDES[arm]}/tool0"

    def relative_pose_topic(self, arm):
        if self.relative_pose_topic_template:
            return self.relative_pose_topic_template.format(
                robot_name=self.robot_name,
                arm=arm,
                prefix=SIDES[arm],
            )
        return (
            f"/{self.robot_name}/{SIDES[arm]}/virtual_object_tcp_transform_node/"
            "relative_object_to_tcp_pose"
        )

    def lookup_tcp(self, arm):
        deadline = time.monotonic() + self.wait_timeout
        last_error = None
        latest_time = Time(seconds=0, nanoseconds=0)
        frame = self.tcp_frame(arm)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.world_frame,
                    frame,
                    latest_time,
                )
                p = transform.transform.translation
                q = transform.transform.rotation
                return ((p.x, p.y, p.z), normalize_quat((q.x, q.y, q.z, q.w)))
            except TransformException as exc:
                last_error = exc
        raise RuntimeError(f"Timed out waiting for TF {self.world_frame} -> {frame}: {last_error}")

    def run(self):
        warmup_deadline = time.monotonic() + 0.25
        while rclpy.ok() and time.monotonic() < warmup_deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

        world_from_tcp = {arm: self.lookup_tcp(arm) for arm in self.arms}
        world_from_object = average_pose(list(world_from_tcp.values()))
        object_from_world = transform_inverse(world_from_object)
        object_from_tcp = {
            arm: transform_multiply(object_from_world, tcp_transform)
            for arm, tcp_transform in world_from_tcp.items()
        }

        object_msg = pose_msg(self.world_frame, self.get_clock().now().to_msg(), world_from_object)
        relative_msgs = {
            arm: pose_msg("virtual_object/base_link", self.get_clock().now().to_msg(), transform)
            for arm, transform in object_from_tcp.items()
        }

        end_time = time.monotonic() + self.publish_duration
        period = 1.0 / self.publish_rate_hz
        while rclpy.ok() and time.monotonic() < end_time:
            stamp = self.get_clock().now().to_msg()
            object_msg.header.stamp = stamp
            self.object_pose_pub.publish(object_msg)
            for arm, msg in relative_msgs.items():
                msg.header.stamp = stamp
                self.relative_pose_pubs[arm].publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

        p, q = world_from_object
        self.get_logger().info(
            "Set virtual object center from arms=%s in %s: "
            "position=[%.4f, %.4f, %.4f], quaternion=[%.5f, %.5f, %.5f, %.5f]"
            % (",".join(self.arms), self.world_frame, p[0], p[1], p[2], q[0], q[1], q[2], q[3])
        )
        for arm, transform in object_from_tcp.items():
            rp, rq = transform
            self.get_logger().info(
                "Published object->%s relative pose: t=[%.4f, %.4f, %.4f], "
                "q=[%.5f, %.5f, %.5f, %.5f] -> %s"
                % (
                    SIDES[arm],
                    rp[0],
                    rp[1],
                    rp[2],
                    rq[0],
                    rq[1],
                    rq[2],
                    rq[3],
                    self.relative_pose_topic(arm),
                )
            )


def main():
    rclpy.init()
    node = SetVirtualObjectFromManipulators()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

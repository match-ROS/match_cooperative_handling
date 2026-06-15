#!/usr/bin/env python3
"""Log cooperative virtual-object target tracking for selected arms."""

import csv
import json
import math
import os
import statistics
import time
from collections import defaultdict

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


SIDES = {"r": "UR10_r", "l": "UR10_l"}


def split_arms(value):
    if isinstance(value, (list, tuple)):
        parts = value
    else:
        parts = str(value).replace(";", ",").split(",")
    arms = []
    for part in parts:
        arm = str(part).strip().lower()
        if arm in SIDES and arm not in arms:
            arms.append(arm)
    return arms or ["r", "l"]


def normalize_quat(q):
    norm = math.sqrt(sum(component * component for component in q))
    if norm <= 1.0e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(component / norm for component in q)


def quat_inverse(q):
    x, y, z, w = normalize_quat(q)
    return (-x, -y, -z, w)


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return normalize_quat(
        (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )
    )


def quat_to_rotvec(q):
    x, y, z, w = normalize_quat(q)
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w
    w = max(-1.0, min(1.0, w))
    angle = 2.0 * math.acos(w)
    scale = math.sqrt(max(0.0, 1.0 - w * w))
    if scale < 1.0e-9:
        return (0.0, 0.0, 0.0)
    return (x / scale * angle, y / scale * angle, z / scale * angle)


def pose_to_tuple(pose):
    return (
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )


def transform_to_tuple(transform):
    return (
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w,
    )


def finite_or_none(value):
    return value if value is not None and math.isfinite(value) else None


class CooperativeTrackingLogger(Node):
    def __init__(self):
        super().__init__("cooperative_tracking_logger")
        self.robot_name = self.declare_parameter("robot_name", "mur620").value
        self.arms = split_arms(self.declare_parameter("arms", "r,l").value)
        self.duration = float(self.declare_parameter("duration", 180.0).value)
        self.sample_rate_hz = float(self.declare_parameter("sample_rate_hz", 50.0).value)
        self.output_dir = os.path.expanduser(
            self.declare_parameter(
                "output_dir", "~/cooperative_tracking_logs"
            ).value
        )
        self.object_pose_topic = self.declare_parameter(
            "object_pose_topic", "/virtual_object/object_pose"
        ).value
        self.object_pose = None
        self.target_poses = {}
        self.reported_errors = {}
        self.last_tf_warning_time = {}

        os.makedirs(self.output_dir, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        arms_token = "".join(self.arms)
        self.csv_path = os.path.join(
            self.output_dir,
            f"cooperative_tracking_{self.robot_name}_{arms_token}_{stamp}.csv",
        )
        self.summary_path = os.path.join(
            self.output_dir,
            f"cooperative_tracking_{self.robot_name}_{arms_token}_{stamp}_summary.json",
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(PoseStamped, self.object_pose_topic, self._object_pose_cb, 10)

        for arm in self.arms:
            prefix = SIDES[arm]
            self.create_subscription(
                PoseStamped,
                f"/{self.robot_name}/{prefix}/virtual_object_tcp_transform_node/target_tcp_pose",
                lambda msg, side=arm: self._target_pose_cb(side, msg),
                10,
            )
            self.create_subscription(
                TwistStamped,
                f"/{self.robot_name}/{prefix}/virtual_object_tcp_transform_node/pose_error",
                lambda msg, side=arm: self._pose_error_cb(side, msg),
                10,
            )

        self.get_logger().info(f"Tracking arms: {','.join(self.arms)}")
        self.get_logger().info(f"Writing CSV log to {self.csv_path}")
        self.get_logger().info(f"Writing summary to {self.summary_path}")

    def _object_pose_cb(self, msg):
        self.object_pose = msg

    def _target_pose_cb(self, arm, msg):
        self.target_poses[arm] = msg

    def _pose_error_cb(self, arm, msg):
        linear = msg.twist.linear
        angular = msg.twist.angular
        self.reported_errors[arm] = {
            "pos": math.sqrt(linear.x * linear.x + linear.y * linear.y + linear.z * linear.z),
            "ori": math.sqrt(
                angular.x * angular.x + angular.y * angular.y + angular.z * angular.z
            ),
            "x": linear.x,
            "y": linear.y,
            "z": linear.z,
            "rx": angular.x,
            "ry": angular.y,
            "rz": angular.z,
        }

    def lookup_actual_pose(self, arm):
        prefix = SIDES[arm]
        base = f"{self.robot_name}/{prefix}/base_link"
        tip = f"{self.robot_name}/{prefix}/tool0"
        transform = self.tf_buffer.lookup_transform(base, tip, Time())
        return transform_to_tuple(transform.transform)

    def compute_error(self, target, actual):
        dx = target[0] - actual[0]
        dy = target[1] - actual[1]
        dz = target[2] - actual[2]
        pos_norm = math.sqrt(dx * dx + dy * dy + dz * dz)
        target_q = target[3:7]
        actual_q = actual[3:7]
        error_q = quat_multiply(target_q, quat_inverse(actual_q))
        rx, ry, rz = quat_to_rotvec(error_q)
        ori_norm = math.sqrt(rx * rx + ry * ry + rz * rz)
        return dx, dy, dz, pos_norm, rx, ry, rz, ori_norm

    def run(self):
        rows = []
        stats = defaultdict(lambda: {"pos": [], "ori": [], "reported_pos": [], "reported_ori": []})
        start_time = self.get_clock().now()
        period = 1.0 / max(1.0, self.sample_rate_hz)
        with open(self.csv_path, "w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(
                csv_file,
                fieldnames=[
                    "time_sec",
                    "arm",
                    "target_x",
                    "target_y",
                    "target_z",
                    "actual_x",
                    "actual_y",
                    "actual_z",
                    "error_x",
                    "error_y",
                    "error_z",
                    "position_error_norm",
                    "orientation_error_rx",
                    "orientation_error_ry",
                    "orientation_error_rz",
                    "orientation_error_norm_rad",
                    "reported_position_error_norm",
                    "reported_orientation_error_norm_rad",
                    "tf_ok",
                    "target_ok",
                ],
            )
            writer.writeheader()
            while rclpy.ok():
                now = self.get_clock().now()
                elapsed = (now - start_time).nanoseconds * 1.0e-9
                if self.duration > 0.0 and elapsed > self.duration:
                    break
                rclpy.spin_once(self, timeout_sec=0.0)
                for arm in self.arms:
                    target_msg = self.target_poses.get(arm)
                    target = pose_to_tuple(target_msg.pose) if target_msg is not None else None
                    actual = None
                    tf_ok = False
                    try:
                        actual = self.lookup_actual_pose(arm)
                        tf_ok = True
                    except TransformException as exc:
                        last_warn = self.last_tf_warning_time.get(arm, -1.0e9)
                        if elapsed - last_warn > 2.0:
                            self.get_logger().warn(f"TF lookup failed for {SIDES[arm]}: {exc}")
                            self.last_tf_warning_time[arm] = elapsed
                    error = (None,) * 8
                    if target is not None and actual is not None:
                        error = self.compute_error(target, actual)
                        stats[arm]["pos"].append(error[3])
                        stats[arm]["ori"].append(error[7])
                    reported = self.reported_errors.get(arm, {})
                    if "pos" in reported:
                        stats[arm]["reported_pos"].append(reported["pos"])
                    if "ori" in reported:
                        stats[arm]["reported_ori"].append(reported["ori"])
                    row = {
                        "time_sec": elapsed,
                        "arm": arm,
                        "target_x": target[0] if target else None,
                        "target_y": target[1] if target else None,
                        "target_z": target[2] if target else None,
                        "actual_x": actual[0] if actual else None,
                        "actual_y": actual[1] if actual else None,
                        "actual_z": actual[2] if actual else None,
                        "error_x": error[0],
                        "error_y": error[1],
                        "error_z": error[2],
                        "position_error_norm": error[3],
                        "orientation_error_rx": error[4],
                        "orientation_error_ry": error[5],
                        "orientation_error_rz": error[6],
                        "orientation_error_norm_rad": error[7],
                        "reported_position_error_norm": finite_or_none(reported.get("pos")),
                        "reported_orientation_error_norm_rad": finite_or_none(reported.get("ori")),
                        "tf_ok": tf_ok,
                        "target_ok": target is not None,
                    }
                    writer.writerow(row)
                    rows.append(row)
                csv_file.flush()
                time.sleep(period)
        self.write_summary(stats, rows)
        return 0

    def metric_summary(self, values):
        if not values:
            return {"samples": 0}
        return {
            "samples": len(values),
            "mean": statistics.fmean(values),
            "rms": math.sqrt(statistics.fmean([value * value for value in values])),
            "max": max(values),
        }

    def write_summary(self, stats, rows):
        summary = {
            "robot_name": self.robot_name,
            "arms": self.arms,
            "csv_path": self.csv_path,
            "duration_sec": rows[-1]["time_sec"] if rows else 0.0,
            "arms_summary": {},
        }
        for arm in self.arms:
            summary["arms_summary"][arm] = {
                "position_error_norm_m": self.metric_summary(stats[arm]["pos"]),
                "orientation_error_norm_rad": self.metric_summary(stats[arm]["ori"]),
                "reported_position_error_norm_m": self.metric_summary(
                    stats[arm]["reported_pos"]
                ),
                "reported_orientation_error_norm_rad": self.metric_summary(
                    stats[arm]["reported_ori"]
                ),
            }
        with open(self.summary_path, "w", encoding="utf-8") as summary_file:
            json.dump(summary, summary_file, indent=2)
        self.get_logger().info(f"Summary written to {self.summary_path}")
        for arm, arm_summary in summary["arms_summary"].items():
            pos = arm_summary["position_error_norm_m"]
            ori = arm_summary["orientation_error_norm_rad"]
            self.get_logger().info(
                f"{SIDES[arm]} error: pos_mean={pos.get('mean', float('nan')):.4f} m, "
                f"pos_max={pos.get('max', float('nan')):.4f} m, "
                f"ori_mean={math.degrees(ori.get('mean', float('nan'))):.2f} deg, "
                f"ori_max={math.degrees(ori.get('max', float('nan'))):.2f} deg"
            )


def main():
    rclpy.init()
    node = CooperativeTrackingLogger()
    try:
        raise SystemExit(node.run())
    except KeyboardInterrupt:
        raise SystemExit(130)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

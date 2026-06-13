#!/usr/bin/env python3
"""Move one MuR620 UR arm to a MoveIt named pose."""

import os
import sys
import time
import traceback
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder
from rclpy.node import Node


SIDES = {"r": "UR_arm_r", "l": "UR_arm_l"}


def arm_controller_config(controller_namespace):
    controller_prefix = f"/{controller_namespace}" if controller_namespace else ""
    left_controller = f"{controller_prefix}/moveit_joint_trajectory_controller_l"
    right_controller = f"{controller_prefix}/moveit_joint_trajectory_controller_r"
    left_lift_controller = f"{controller_prefix}/moveit_joint_trajectory_controller_lift_l"
    right_lift_controller = f"{controller_prefix}/moveit_joint_trajectory_controller_lift_r"
    left_joints = [
        "UR10_l/shoulder_pan_joint",
        "UR10_l/shoulder_lift_joint",
        "UR10_l/elbow_joint",
        "UR10_l/wrist_1_joint",
        "UR10_l/wrist_2_joint",
        "UR10_l/wrist_3_joint",
    ]
    right_joints = [
        "UR10_r/shoulder_pan_joint",
        "UR10_r/shoulder_lift_joint",
        "UR10_r/elbow_joint",
        "UR10_r/wrist_1_joint",
        "UR10_r/wrist_2_joint",
        "UR10_r/wrist_3_joint",
    ]
    return {
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "trajectory_execution": {
            "allowed_execution_duration_scaling": 1.2,
            "allowed_goal_duration_margin": 0.5,
            "allowed_start_tolerance": 0.01,
            "execution_duration_monitoring": False,
        },
        "moveit_simple_controller_manager": {
            "controller_names": [
                left_controller,
                right_controller,
                left_lift_controller,
                right_lift_controller,
            ],
            left_controller: {
                "action_ns": "follow_joint_trajectory",
                "type": "FollowJointTrajectory",
                "default": True,
                "joints": left_joints,
            },
            right_controller: {
                "action_ns": "follow_joint_trajectory",
                "type": "FollowJointTrajectory",
                "default": True,
                "joints": right_joints,
            },
            left_lift_controller: {
                "action_ns": "follow_joint_trajectory",
                "type": "FollowJointTrajectory",
                "default": True,
                "joints": ["left_lift_joint"] + left_joints,
            },
            right_lift_controller: {
                "action_ns": "follow_joint_trajectory",
                "type": "FollowJointTrajectory",
                "default": True,
                "joints": ["right_lift_joint"] + right_joints,
            },
        },
    }


def load_robot_profile(robot_profile):
    mur_launch_hardware_path = get_package_share_directory("mur_launch_hardware")
    profile_file = os.path.join(mur_launch_hardware_path, "config", "mur_robot_profiles.yaml")
    with open(profile_file, "r", encoding="utf-8") as handle:
        profiles = yaml.safe_load(handle) or {}
    robots = profiles.get("robots", {})
    if robot_profile not in robots:
        raise RuntimeError(f"Robot profile '{robot_profile}' not found in {profile_file}")
    return robots[robot_profile], mur_launch_hardware_path


def resolve_profile_file(mur_launch_hardware_path, path):
    if not path:
        return ""
    if os.path.isabs(path):
        return path
    return os.path.join(mur_launch_hardware_path, path)


def robot_description_source(robot_name, robot_profile, ur_type):
    mur_description_path = get_package_share_directory("mur_description")
    xacro_file = os.path.join(mur_description_path, "urdf", "mur_620.gazebo.xacro")
    profile, mur_launch_hardware_path = load_robot_profile(robot_profile)
    arms = profile.get("arms", {})
    left = arms.get("l", {})
    right = arms.get("r", {})
    return xacro_file, {
        "tf_prefix": robot_name,
        "tf_prefix_mir": robot_name,
        "robot_namespace": robot_name,
        "use_arms": "true",
        "use_camera": "true",
        "use_lidar": "true",
        "use_lift": "true" if profile.get("use_lift", True) else "false",
        "use_simple_collisions": "false",
        "use_simple_visuals": "false",
        "use_high_quality_visuals": "false",
        "use_base_visual_mesh": "false",
        "use_top_visual_mesh": "false",
        "use_wheel_visual_mesh": "false",
        "use_caster_visual_mesh": "false",
        "use_lift_visual_mesh": "false",
        "use_laser_visual_mesh": "false",
        "ur_type": ur_type,
        "ur_l_xyz": left.get("mount_xyz", "0 0 0"),
        "ur_l_rpy": left.get("mount_rpy", "0 0 0"),
        "ur_r_xyz": right.get("mount_xyz", "0 0 0"),
        "ur_r_rpy": right.get("mount_rpy", "0 0 3.14159265359"),
        "kinematics_params_l": resolve_profile_file(
            mur_launch_hardware_path, left.get("kinematics_params_file", "")
        ),
        "kinematics_params_r": resolve_profile_file(
            mur_launch_hardware_path, right.get("kinematics_params_file", "")
        ),
    }


class MoveArmToNamedPose(Node):
    def __init__(self):
        super().__init__("move_arm_to_named_pose")
        self.declare_parameter("robot_name", "mur620")
        self.declare_parameter("robot_profile", "mur620d")
        self.declare_parameter("ur_type", "ur10")
        self.declare_parameter("arm", "r")
        self.declare_parameter("group", "")
        self.declare_parameter("named_pose", "Home_custom")
        self.declare_parameter("node_name", "cooperative_home_moveit_py")
        self.declare_parameter("wait_after_init", 1.0)

        self.robot_name = str(self.get_parameter("robot_name").value)
        self.robot_profile = str(self.get_parameter("robot_profile").value)
        self.ur_type = str(self.get_parameter("ur_type").value)
        self.arm = str(self.get_parameter("arm").value)
        self.group = str(self.get_parameter("group").value) or SIDES.get(self.arm, "UR_arm_r")
        self.named_pose = str(self.get_parameter("named_pose").value)
        self.node_name = str(self.get_parameter("node_name").value)
        self.wait_after_init = float(self.get_parameter("wait_after_init").value)

    def make_moveit_config(self):
        robot_xacro_file, robot_xacro_mappings = robot_description_source(
            self.robot_name, self.robot_profile, self.ur_type
        )
        virtual_joint_parent_frame = f"{self.robot_name}/base_footprint"
        moveit_config = (
            MoveItConfigsBuilder(robot_name="mur620", package_name="mur_moveit_config")
            .robot_description(robot_xacro_file, robot_xacro_mappings)
            .robot_description_semantic(
                Path("srdf") / "mur620.srdf.xacro",
                {
                    "prefix": "UR10",
                    "model_name": "mur620",
                    "virtual_joint_parent_frame": virtual_joint_parent_frame,
                },
            )
            .moveit_cpp(file_path="config/moveit_cpp.yaml")
            .to_moveit_configs()
            .to_dict()
        )
        moveit_config.update(arm_controller_config(self.robot_name))
        moveit_config["use_sim_time"] = False
        return moveit_config

    def run(self):
        if self.arm not in SIDES:
            self.get_logger().error(f"arm must be 'l' or 'r', got '{self.arm}'")
            return 2

        self.get_logger().info(
            f"Planning {self.group} ({self.arm}) to named pose '{self.named_pose}' "
            f"with profile '{self.robot_profile}'"
        )
        moveit = MoveItPy(node_name=self.node_name, config_dict=self.make_moveit_config())
        if self.wait_after_init > 0.0:
            time.sleep(self.wait_after_init)

        planning_component = moveit.get_planning_component(self.group)
        if not planning_component.set_goal_state(self.named_pose):
            self.get_logger().error(
                f"Failed to set goal state '{self.named_pose}' for group '{self.group}'"
            )
            return 3

        plan_result = planning_component.plan()
        error_code = getattr(plan_result.error_code, "val", 999)
        if error_code != 1:
            self.get_logger().error(
                f"Planning failed for group '{self.group}' to '{self.named_pose}' "
                f"with error code {error_code}"
            )
            return 4

        self.get_logger().info("Planning succeeded; executing trajectory...")
        execute_result = moveit.execute(plan_result.trajectory, controllers=[])
        self.get_logger().info(
            f"Execution request finished for {self.group} -> {self.named_pose}; "
            f"result={execute_result}"
        )
        return 0


def main():
    rclpy.init()
    node = MoveArmToNamedPose()
    try:
        exit_code = node.run()
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(f"Home move failed: {exc}\n{traceback.format_exc()}")
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()

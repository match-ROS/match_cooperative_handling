#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_broadcaster.h>

namespace match_cooperative_handling
{
namespace
{

tf2::Vector3 vector_from_msg(const geometry_msgs::msg::Vector3 & msg)
{
  return {msg.x, msg.y, msg.z};
}

void vector_to_msg(const tf2::Vector3 & value, geometry_msgs::msg::Vector3 & msg)
{
  msg.x = value.x();
  msg.y = value.y();
  msg.z = value.z();
}

void vector_to_msg(const tf2::Vector3 & value, geometry_msgs::msg::Point & msg)
{
  msg.x = value.x();
  msg.y = value.y();
  msg.z = value.z();
}

void quaternion_to_msg(const tf2::Quaternion & value, geometry_msgs::msg::Quaternion & msg)
{
  tf2::Quaternion q = value;
  q.normalize();
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  msg.w = q.w();
}

tf2::Quaternion quaternion_from_msg(const geometry_msgs::msg::Quaternion & msg)
{
  tf2::Quaternion q(msg.x, msg.y, msg.z, msg.w);
  if (q.length2() <= 1.0e-12) {
    q.setValue(0.0, 0.0, 0.0, 1.0);
  }
  q.normalize();
  return q;
}

tf2::Quaternion quaternion_from_rotation_vector(const tf2::Vector3 & rotation_vector)
{
  const double angle = rotation_vector.length();
  tf2::Quaternion q;
  if (angle <= 1.0e-12) {
    q.setValue(0.0, 0.0, 0.0, 1.0);
  } else {
    q.setRotation(rotation_vector.normalized(), angle);
  }
  q.normalize();
  return q;
}

tf2::Transform transform_from_pose(const geometry_msgs::msg::Pose & pose)
{
  tf2::Transform transform;
  transform.setOrigin(tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
  transform.setRotation(quaternion_from_msg(pose.orientation));
  return transform;
}

}  // namespace

class VirtualObjectStateNode : public rclcpp::Node
{
public:
  VirtualObjectStateNode()
  : Node("virtual_object_state_node")
  {
    world_frame_ = declare_parameter<std::string>("world_frame", "mur620/base_link");
    object_frame_ = declare_parameter<std::string>("object_frame", "virtual_object/base_link");
    twist_cmd_topic_ =
      declare_parameter<std::string>("twist_cmd_topic", "/virtual_object/object_twist_cmd");
    set_pose_topic_ = declare_parameter<std::string>("set_pose_topic", "/virtual_object/set_pose");
    object_pose_topic_ =
      declare_parameter<std::string>("object_pose_topic", "/virtual_object/object_pose");
    object_twist_topic_ =
      declare_parameter<std::string>("object_twist_topic", "/virtual_object/object_twist");
    rate_hz_ = std::max(1.0, declare_parameter<double>("rate", 500.0));
    cmd_timeout_ = std::max(0.0, declare_parameter<double>("cmd_timeout", 0.15));

    object_pose_.setIdentity();
    current_twist_.header.frame_id = world_frame_;
    last_update_time_ = now();

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(object_pose_topic_, 10);
    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(object_twist_topic_, 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      twist_cmd_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        const std::string frame = msg->header.frame_id.empty() ? world_frame_ : msg->header.frame_id;
        if (frame != world_frame_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Ignoring object twist in frame '%s'; expected '%s' or empty",
            frame.c_str(), world_frame_.c_str());
          return;
        }
        current_twist_ = *msg;
        current_twist_.header.frame_id = world_frame_;
        last_cmd_time_ = now();
        have_cmd_ = true;
      });

    auto set_pose_qos = rclcpp::QoS(1).reliable().transient_local();
    set_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      set_pose_topic_, set_pose_qos,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        const std::string frame = msg->header.frame_id.empty() ? world_frame_ : msg->header.frame_id;
        if (frame != world_frame_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Ignoring object set_pose in frame '%s'; expected '%s' or empty",
            frame.c_str(), world_frame_.c_str());
          return;
        }
        object_pose_ = transform_from_pose(msg->pose);
        RCLCPP_INFO(
          get_logger(), "Set virtual object pose in %s: [%.3f, %.3f, %.3f]",
          world_frame_.c_str(), object_pose_.getOrigin().x(), object_pose_.getOrigin().y(),
          object_pose_.getOrigin().z());
      });

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / rate_hz_));
    timer_ = create_wall_timer(period, [this]() { update(); });

    RCLCPP_INFO(
      get_logger(), "Virtual object state node: %s -> %s, cmd=%s, set_pose=%s",
      world_frame_.c_str(), object_frame_.c_str(), twist_cmd_topic_.c_str(),
      set_pose_topic_.c_str());
  }

private:
  void update()
  {
    const rclcpp::Time stamp = now();
    double dt = (stamp - last_update_time_).seconds();
    last_update_time_ = stamp;
    if (!std::isfinite(dt) || dt < 0.0 || dt > 0.1) {
      dt = 1.0 / rate_hz_;
    }

    geometry_msgs::msg::TwistStamped active_twist = current_twist_;
    active_twist.header.frame_id = world_frame_;
    active_twist.header.stamp = stamp;
    if (!have_cmd_ || (stamp - last_cmd_time_).seconds() > cmd_timeout_) {
      active_twist.twist = geometry_msgs::msg::Twist();
    }

    const tf2::Vector3 linear = vector_from_msg(active_twist.twist.linear);
    const tf2::Vector3 angular = vector_from_msg(active_twist.twist.angular);
    object_pose_.setOrigin(object_pose_.getOrigin() + linear * dt);
    tf2::Quaternion rotation = object_pose_.getRotation();
    rotation = quaternion_from_rotation_vector(angular * dt) * rotation;
    rotation.normalize();
    object_pose_.setRotation(rotation);

    publish_state(stamp, active_twist);
  }

  void publish_state(
    const rclcpp::Time & stamp,
    const geometry_msgs::msg::TwistStamped & active_twist)
  {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = world_frame_;
    vector_to_msg(object_pose_.getOrigin(), pose_msg.pose.position);
    quaternion_to_msg(object_pose_.getRotation(), pose_msg.pose.orientation);
    pose_pub_->publish(pose_msg);
    twist_pub_->publish(active_twist);

    geometry_msgs::msg::TransformStamped transform_msg;
    transform_msg.header.stamp = stamp;
    transform_msg.header.frame_id = world_frame_;
    transform_msg.child_frame_id = object_frame_;
    vector_to_msg(object_pose_.getOrigin(), transform_msg.transform.translation);
    quaternion_to_msg(object_pose_.getRotation(), transform_msg.transform.rotation);
    tf_broadcaster_->sendTransform(transform_msg);
  }

  std::string world_frame_;
  std::string object_frame_;
  std::string twist_cmd_topic_;
  std::string set_pose_topic_;
  std::string object_pose_topic_;
  std::string object_twist_topic_;
  double rate_hz_{500.0};
  double cmd_timeout_{0.15};

  tf2::Transform object_pose_{tf2::Transform::getIdentity()};
  geometry_msgs::msg::TwistStamped current_twist_;
  bool have_cmd_{false};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr set_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace match_cooperative_handling

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<match_cooperative_handling::VirtualObjectStateNode>());
  rclcpp::shutdown();
  return 0;
}

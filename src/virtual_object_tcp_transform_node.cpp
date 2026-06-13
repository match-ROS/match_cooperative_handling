#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace match_cooperative_handling
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

std::string arm_prefix(const std::string & arm)
{
  return arm == "l" ? "UR10_l" : "UR10_r";
}

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

tf2::Quaternion quaternion_from_msg(const geometry_msgs::msg::Quaternion & msg)
{
  tf2::Quaternion q(msg.x, msg.y, msg.z, msg.w);
  if (q.length2() <= 1.0e-12) {
    q.setValue(0.0, 0.0, 0.0, 1.0);
  }
  q.normalize();
  return q;
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

tf2::Transform transform_from_pose(const geometry_msgs::msg::Pose & pose)
{
  tf2::Transform transform;
  transform.setOrigin(tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
  transform.setRotation(quaternion_from_msg(pose.orientation));
  return transform;
}

tf2::Transform transform_from_msg(const geometry_msgs::msg::TransformStamped & msg)
{
  tf2::Transform transform;
  transform.setOrigin(tf2::Vector3(
      msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z));
  transform.setRotation(quaternion_from_msg(msg.transform.rotation));
  return transform;
}

tf2::Vector3 rotation_vector_from_quaternion(tf2::Quaternion q)
{
  q.normalize();
  if (q.w() < 0.0) {
    q = tf2::Quaternion(-q.x(), -q.y(), -q.z(), -q.w());
  }
  const double angle = q.getAngle();
  if (angle <= 1.0e-12) {
    return {0.0, 0.0, 0.0};
  }
  return q.getAxis() * angle;
}

tf2::Vector3 clamp_norm(const tf2::Vector3 & value, double limit)
{
  const double abs_limit = std::abs(limit);
  const double norm = value.length();
  if (abs_limit <= 0.0 || norm <= abs_limit || norm <= 1.0e-12) {
    return value;
  }
  return value * (abs_limit / norm);
}

}  // namespace

class VirtualObjectTcpTransformNode : public rclcpp::Node
{
public:
  VirtualObjectTcpTransformNode()
  : Node("virtual_object_tcp_transform_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    robot_name_ = declare_parameter<std::string>("robot_name", "mur620");
    arm_ = declare_parameter<std::string>("arm", "r");
    prefix_ = declare_parameter<std::string>("prefix", arm_prefix(arm_));
    world_frame_ = declare_parameter<std::string>("world_frame", robot_name_ + "/base_link");
    base_frame_ = declare_parameter<std::string>(
      "base_frame", robot_name_ + "/" + prefix_ + "/base_link");
    tcp_frame_ = declare_parameter<std::string>(
      "tcp_frame", robot_name_ + "/" + prefix_ + "/tool0");
    command_frame_ = declare_parameter<std::string>("command_frame", prefix_ + "/base_link");
    controller_name_ = declare_parameter<std::string>(
      "controller_name", "integrated_cartesian_admittance_controller");
    controller_twist_topic_ = declare_parameter<std::string>(
      "controller_twist_topic",
      "/" + robot_name_ + "/" + prefix_ + "/" + controller_name_ + "/equilibrium_twist_cmd");
    object_pose_topic_ =
      declare_parameter<std::string>("object_pose_topic", "/virtual_object/object_pose");
    object_twist_topic_ =
      declare_parameter<std::string>("object_twist_topic", "/virtual_object/object_twist");
    relative_pose_topic_ =
      declare_parameter<std::string>("relative_pose_topic", "~/relative_object_to_tcp_pose");
    target_frame_ = declare_parameter<std::string>(
      "target_frame", robot_name_ + "/" + prefix_ + "/virtual_object_target_tcp");
    rate_hz_ = std::max(1.0, declare_parameter<double>("rate", 500.0));
    input_timeout_ = std::max(0.0, declare_parameter<double>("input_timeout", 0.25));
    position_gain_ = std::max(0.0, declare_parameter<double>("position_gain", 1.0));
    orientation_gain_ = std::max(0.0, declare_parameter<double>("orientation_gain", 0.8));
    max_linear_velocity_ = std::max(0.0, declare_parameter<double>("max_linear_velocity", 0.12));
    max_angular_velocity_ = std::max(0.0, declare_parameter<double>("max_angular_velocity", 0.4));
    start_max_position_error_ =
      std::max(0.0, declare_parameter<double>("start_max_position_error", 0.03));
    start_max_orientation_error_ =
      std::max(0.0, declare_parameter<double>("start_max_orientation_error_deg", 5.0)) *
      kPi / 180.0;
    require_controller_subscriber_ =
      declare_parameter<bool>("require_controller_subscriber", true);
    status_publish_rate_hz_ =
      std::max(0.1, declare_parameter<double>("status_publish_rate_hz", 10.0));
    publish_tf_ = declare_parameter<bool>("publish_tf", true);

    command_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(controller_twist_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("~/status", 10);
    target_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/target_tcp_pose", 10);
    relative_pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>("~/relative_object_to_tcp_pose_debug", 10);
    pose_error_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("~/pose_error", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    object_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      object_pose_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        const std::string frame = msg->header.frame_id.empty() ? world_frame_ : msg->header.frame_id;
        if (frame != world_frame_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Ignoring object pose in frame '%s'; expected '%s' or empty",
            frame.c_str(), world_frame_.c_str());
          return;
        }
        object_pose_ = transform_from_pose(msg->pose);
        object_pose_stamp_ =
          rclcpp::Time(msg->header.stamp).nanoseconds() == 0 ? now() : rclcpp::Time(msg->header.stamp);
        have_object_pose_ = true;
      });

    object_twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      object_twist_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        const std::string frame = msg->header.frame_id.empty() ? world_frame_ : msg->header.frame_id;
        if (frame != world_frame_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Ignoring object twist in frame '%s'; expected '%s' or empty",
            frame.c_str(), world_frame_.c_str());
          return;
        }
        object_twist_ = *msg;
        object_twist_.header.frame_id = world_frame_;
        object_twist_stamp_ =
          rclcpp::Time(msg->header.stamp).nanoseconds() == 0 ? now() : rclcpp::Time(msg->header.stamp);
        have_object_twist_ = true;
      });

    auto relative_qos = rclcpp::QoS(1).reliable().transient_local();
    relative_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      relative_pose_topic_, relative_qos,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        relative_object_to_tcp_ = transform_from_pose(msg->pose);
        relative_pose_stamp_ = now();
        have_relative_pose_ = true;
        RCLCPP_INFO(
          get_logger(), "Updated object->tcp relative pose: [%.3f, %.3f, %.3f]",
          relative_object_to_tcp_.getOrigin().x(), relative_object_to_tcp_.getOrigin().y(),
          relative_object_to_tcp_.getOrigin().z());
      });

    start_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/start",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        std::string reason;
        double position_error = 0.0;
        double orientation_error = 0.0;
        if (!start_preflight(reason, position_error, orientation_error)) {
          armed_ = false;
          response->success = false;
          response->message = reason;
          publish_zero(now());
          publish_status(now(), "blocked: " + reason, true);
          return;
        }
        armed_ = true;
        response->success = true;
        response->message =
          "armed: position_error=" + std::to_string(position_error) +
          " m, orientation_error=" + std::to_string(orientation_error * 180.0 / kPi) + " deg";
        publish_status(now(), "armed", true);
      });

    stop_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/stop",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        armed_ = false;
        publish_zero(now());
        response->success = true;
        response->message = "disarmed";
        publish_status(now(), "disarmed", true);
      });

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / rate_hz_));
    timer_ = create_wall_timer(period, [this]() { update(); });

    RCLCPP_INFO(
      get_logger(), "Virtual object TCP transform node for %s: object=%s, command=%s",
      prefix_.c_str(), object_pose_topic_.c_str(), controller_twist_topic_.c_str());
  }

private:
  void update()
  {
    const rclcpp::Time stamp = now();
    std::string reason;
    if (!inputs_ready(stamp, reason)) {
      armed_ = false;
      publish_zero(stamp);
      publish_status(stamp, "blocked: " + reason, false);
      return;
    }

    tf2::Transform base_from_world;
    tf2::Transform current_tcp;
    try {
      base_from_world = transform_from_msg(tf_buffer_.lookupTransform(
          base_frame_, world_frame_, tf2::TimePointZero));
      current_tcp = transform_from_msg(tf_buffer_.lookupTransform(
          base_frame_, tcp_frame_, tf2::TimePointZero));
    } catch (const tf2::TransformException & exc) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Waiting for TF %s->%s and %s->%s: %s",
        base_frame_.c_str(), world_frame_.c_str(), base_frame_.c_str(), tcp_frame_.c_str(),
        exc.what());
      publish_zero(stamp);
      publish_status(stamp, "blocked: waiting for TF", false);
      return;
    }

    const tf2::Transform target_world = object_pose_ * relative_object_to_tcp_;
    const tf2::Transform target_base = base_from_world * target_world;
    const tf2::Vector3 position_error = target_base.getOrigin() - current_tcp.getOrigin();
    const tf2::Quaternion orientation_error =
      target_base.getRotation() * current_tcp.getRotation().inverse();
    const tf2::Vector3 rotation_error = rotation_vector_from_quaternion(orientation_error);

    const tf2::Vector3 object_linear_world = vector_from_msg(object_twist_.twist.linear);
    const tf2::Vector3 object_angular_world = vector_from_msg(object_twist_.twist.angular);
    const tf2::Vector3 object_to_tcp_world = target_world.getOrigin() - object_pose_.getOrigin();
    const tf2::Vector3 tcp_linear_world =
      object_linear_world + object_angular_world.cross(object_to_tcp_world);
    const tf2::Vector3 tcp_angular_world = object_angular_world;

    const tf2::Matrix3x3 & rotation_base_from_world = base_from_world.getBasis();
    tf2::Vector3 linear_cmd =
      rotation_base_from_world * tcp_linear_world + position_error * position_gain_;
    tf2::Vector3 angular_cmd =
      rotation_base_from_world * tcp_angular_world + rotation_error * orientation_gain_;
    linear_cmd = clamp_norm(linear_cmd, max_linear_velocity_);
    angular_cmd = clamp_norm(angular_cmd, max_angular_velocity_);

    if (armed_) {
      geometry_msgs::msg::TwistStamped command;
      command.header.stamp = stamp;
      command.header.frame_id = command_frame_;
      vector_to_msg(linear_cmd, command.twist.linear);
      vector_to_msg(angular_cmd, command.twist.angular);
      command_pub_->publish(command);
      publish_status(stamp, "armed", false);
    } else {
      publish_zero(stamp);
      publish_status(stamp, "ready", false);
    }

    publish_debug(stamp, target_base, position_error, rotation_error);
  }

  bool inputs_ready(const rclcpp::Time & stamp, std::string & reason) const
  {
    if (!have_object_pose_) {
      reason = "missing object pose";
      return false;
    }
    if (!have_object_twist_) {
      reason = "missing object twist";
      return false;
    }
    if (!have_relative_pose_) {
      reason = "missing relative object->tcp pose";
      return false;
    }
    if ((stamp - object_pose_stamp_).seconds() > input_timeout_) {
      reason = "stale object pose";
      return false;
    }
    if ((stamp - object_twist_stamp_).seconds() > input_timeout_) {
      reason = "stale object twist";
      return false;
    }
    reason.clear();
    return true;
  }

  bool start_preflight(std::string & reason, double & position_error, double & orientation_error)
  {
    const rclcpp::Time stamp = now();
    if (!inputs_ready(stamp, reason)) {
      return false;
    }
    if (require_controller_subscriber_ && command_pub_->get_subscription_count() == 0) {
      reason = "controller command subscriber missing";
      return false;
    }

    tf2::Transform base_from_world;
    tf2::Transform current_tcp;
    try {
      base_from_world = transform_from_msg(tf_buffer_.lookupTransform(
          base_frame_, world_frame_, tf2::TimePointZero));
      current_tcp = transform_from_msg(tf_buffer_.lookupTransform(
          base_frame_, tcp_frame_, tf2::TimePointZero));
    } catch (const tf2::TransformException & exc) {
      reason = std::string("waiting for TF: ") + exc.what();
      return false;
    }

    const tf2::Transform target_world = object_pose_ * relative_object_to_tcp_;
    const tf2::Transform target_base = base_from_world * target_world;
    const tf2::Vector3 position_delta = target_base.getOrigin() - current_tcp.getOrigin();
    const tf2::Quaternion orientation_delta =
      target_base.getRotation() * current_tcp.getRotation().inverse();
    position_error = position_delta.length();
    orientation_error = rotation_vector_from_quaternion(orientation_delta).length();

    if (position_error > start_max_position_error_) {
      reason =
        "start position error too large: " + std::to_string(position_error) +
        " m > " + std::to_string(start_max_position_error_) + " m";
      return false;
    }
    if (orientation_error > start_max_orientation_error_) {
      reason =
        "start orientation error too large: " +
        std::to_string(orientation_error * 180.0 / kPi) +
        " deg > " + std::to_string(start_max_orientation_error_ * 180.0 / kPi) + " deg";
      return false;
    }
    reason.clear();
    return true;
  }

  void publish_zero(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::TwistStamped command;
    command.header.stamp = stamp;
    command.header.frame_id = command_frame_;
    command_pub_->publish(command);
  }

  void publish_status(
    const rclcpp::Time & stamp,
    const std::string & status,
    bool force)
  {
    if (!force) {
      const double min_period = 1.0 / status_publish_rate_hz_;
      if ((stamp - last_status_publish_time_).seconds() < min_period && status == last_status_) {
        return;
      }
    }
    std_msgs::msg::String msg;
    msg.data = status;
    status_pub_->publish(msg);
    last_status_ = status;
    last_status_publish_time_ = stamp;
  }

  void publish_debug(
    const rclcpp::Time & stamp,
    const tf2::Transform & target_base,
    const tf2::Vector3 & position_error,
    const tf2::Vector3 & rotation_error)
  {
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.stamp = stamp;
    target_pose.header.frame_id = base_frame_;
    vector_to_msg(target_base.getOrigin(), target_pose.pose.position);
    quaternion_to_msg(target_base.getRotation(), target_pose.pose.orientation);
    target_pose_pub_->publish(target_pose);

    geometry_msgs::msg::PoseStamped relative_pose;
    relative_pose.header.stamp = stamp;
    relative_pose.header.frame_id = "virtual_object/base_link";
    vector_to_msg(relative_object_to_tcp_.getOrigin(), relative_pose.pose.position);
    quaternion_to_msg(relative_object_to_tcp_.getRotation(), relative_pose.pose.orientation);
    relative_pose_pub_->publish(relative_pose);

    geometry_msgs::msg::TwistStamped error;
    error.header.stamp = stamp;
    error.header.frame_id = base_frame_;
    vector_to_msg(position_error, error.twist.linear);
    vector_to_msg(rotation_error, error.twist.angular);
    pose_error_pub_->publish(error);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = stamp;
      transform.header.frame_id = base_frame_;
      transform.child_frame_id = target_frame_;
      vector_to_msg(target_base.getOrigin(), transform.transform.translation);
      quaternion_to_msg(target_base.getRotation(), transform.transform.rotation);
      tf_broadcaster_->sendTransform(transform);
    }
  }

  std::string robot_name_;
  std::string arm_;
  std::string prefix_;
  std::string world_frame_;
  std::string base_frame_;
  std::string tcp_frame_;
  std::string command_frame_;
  std::string controller_name_;
  std::string controller_twist_topic_;
  std::string object_pose_topic_;
  std::string object_twist_topic_;
  std::string relative_pose_topic_;
  std::string target_frame_;
  double rate_hz_{500.0};
  double input_timeout_{0.25};
  double position_gain_{1.0};
  double orientation_gain_{0.8};
  double max_linear_velocity_{0.12};
  double max_angular_velocity_{0.4};
  double start_max_position_error_{0.03};
  double start_max_orientation_error_{5.0 * kPi / 180.0};
  double status_publish_rate_hz_{10.0};
  bool require_controller_subscriber_{true};
  bool publish_tf_{true};
  bool armed_{false};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  tf2::Transform object_pose_{tf2::Transform::getIdentity()};
  tf2::Transform relative_object_to_tcp_{tf2::Transform::getIdentity()};
  geometry_msgs::msg::TwistStamped object_twist_;
  bool have_object_pose_{false};
  bool have_object_twist_{false};
  bool have_relative_pose_{false};
  rclcpp::Time object_pose_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time object_twist_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time relative_pose_stamp_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr object_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr object_twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr relative_pose_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr relative_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pose_error_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_status_publish_time_{0, 0, RCL_ROS_TIME};
  std::string last_status_;
};

}  // namespace match_cooperative_handling

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<match_cooperative_handling::VirtualObjectTcpTransformNode>());
  rclcpp::shutdown();
  return 0;
}

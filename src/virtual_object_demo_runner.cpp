#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace match_cooperative_handling
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

struct Segment
{
  std::string label;
  int axis;
  double distance;
  double max_velocity;
};

double smooth_velocity_scale(double u)
{
  u = std::clamp(u, 0.0, 1.0);
  return 30.0 * u * u - 60.0 * u * u * u + 30.0 * u * u * u * u;
}

}  // namespace

class VirtualObjectDemoRunner : public rclcpp::Node
{
public:
  VirtualObjectDemoRunner()
  : Node("virtual_object_demo_runner_cpp")
  {
    robot_name_ = declare_parameter<std::string>("robot_name", "mur620");
    demo_name_ = declare_parameter<std::string>("demo_name", "safe_wiggle");
    world_frame_ = declare_parameter<std::string>("world_frame", robot_name_ + "/base_link");
    twist_topic_ = declare_parameter<std::string>("twist_topic", "/virtual_object/object_twist_cmd");
    object_pose_topic_ =
      declare_parameter<std::string>("object_pose_topic", "/virtual_object/object_pose");
    xy_amplitude_ = declare_parameter<double>("xy_amplitude", 0.05);
    z_lift_ = declare_parameter<double>("z_lift", 0.05);
    yaw_amplitude_deg_ = declare_parameter<double>("yaw_amplitude_deg", 5.0);
    linear_velocity_ = std::max(1.0e-6, declare_parameter<double>("linear_velocity", 0.02));
    angular_velocity_ = std::max(1.0e-6, declare_parameter<double>("angular_velocity", 0.10));
    repetitions_ = std::max(1, static_cast<int>(declare_parameter<int>("repetitions", 1)));
    publish_rate_hz_ = std::max(1.0, declare_parameter<double>("publish_rate_hz", 500.0));
    pose_timeout_ = std::max(0.0, declare_parameter<double>("pose_timeout", 3.0));
    min_segment_duration_ =
      std::max(0.0, declare_parameter<double>("min_segment_duration", 1.0));

    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(twist_topic_, 10);
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      object_pose_topic_, 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr) {
        have_object_pose_ = true;
        last_pose_time_ = now();
      });

    RCLCPP_INFO(
      get_logger(), "Virtual object demo runner ready: demo=%s, rate=%.1f Hz, cmd=%s",
      demo_name_.c_str(), publish_rate_hz_, twist_topic_.c_str());
  }

  int run()
  {
    if (demo_name_ != "safe_wiggle") {
      RCLCPP_ERROR(get_logger(), "Unsupported demo_name '%s'", demo_name_.c_str());
      publish_zero();
      return 2;
    }
    if (!wait_for_object_pose()) {
      publish_zero();
      return 1;
    }

    RCLCPP_INFO(
      get_logger(),
      "Starting Safe Wiggle demo at %.1f Hz. Motion gates must already be armed.",
      publish_rate_hz_);
    for (int repetition = 0; rclcpp::ok() && repetition < repetitions_; ++repetition) {
      RCLCPP_INFO(get_logger(), "Safe Wiggle repetition %d/%d", repetition + 1, repetitions_);
      for (const auto & segment : safe_wiggle_segments()) {
        if (!run_segment(segment)) {
          RCLCPP_WARN(get_logger(), "Demo stopped before completion");
          publish_zero();
          return 130;
        }
      }
    }
    publish_zero();
    RCLCPP_INFO(get_logger(), "Safe Wiggle demo complete");
    return 0;
  }

private:
  bool wait_for_object_pose()
  {
    RCLCPP_INFO(get_logger(), "Waiting for virtual object pose on %s", object_pose_topic_.c_str());
    const auto deadline = now() + rclcpp::Duration::from_seconds(pose_timeout_);
    rclcpp::Rate wait_rate(50.0);
    while (rclcpp::ok() && now() < deadline) {
      rclcpp::spin_some(get_node_base_interface());
      if (have_object_pose_) {
        RCLCPP_INFO(get_logger(), "Virtual object pose is available");
        return true;
      }
      wait_rate.sleep();
    }
    RCLCPP_ERROR(get_logger(), "Timed out waiting for virtual object pose");
    return false;
  }

  std::vector<Segment> safe_wiggle_segments() const
  {
    const double yaw = yaw_amplitude_deg_ * kPi / 180.0;
    return {
      {"lift +Z", 2, z_lift_, linear_velocity_},
      {"X+", 0, xy_amplitude_, linear_velocity_},
      {"X-", 0, -2.0 * xy_amplitude_, linear_velocity_},
      {"X center", 0, xy_amplitude_, linear_velocity_},
      {"Y+", 1, xy_amplitude_, linear_velocity_},
      {"Y-", 1, -2.0 * xy_amplitude_, linear_velocity_},
      {"Y center", 1, xy_amplitude_, linear_velocity_},
      {"Yaw+", 5, yaw, angular_velocity_},
      {"Yaw-", 5, -2.0 * yaw, angular_velocity_},
      {"Yaw center", 5, yaw, angular_velocity_},
      {"lower -Z", 2, -z_lift_, linear_velocity_},
    };
  }

  bool run_segment(const Segment & segment)
  {
    if (std::abs(segment.distance) < 1.0e-12) {
      return true;
    }
    const double duration = std::max(
      min_segment_duration_, 1.875 * std::abs(segment.distance) /
      std::max(std::abs(segment.max_velocity), 1.0e-6));
    RCLCPP_INFO(
      get_logger(), "Segment %s: distance=%.4f, duration=%.2fs",
      segment.label.c_str(), segment.distance, duration);

    const auto start = now();
    rclcpp::Rate rate(publish_rate_hz_);
    while (rclcpp::ok()) {
      rclcpp::spin_some(get_node_base_interface());
      const double elapsed = (now() - start).seconds();
      if (elapsed >= duration) {
        break;
      }
      const double u = elapsed / duration;
      const double velocity = segment.distance / duration * smooth_velocity_scale(u);
      std::array<double, 6> command{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      command[static_cast<std::size_t>(segment.axis)] = velocity;
      publish_twist(command);
      rate.sleep();
    }
    publish_zero();
    return true;
  }

  void publish_zero()
  {
    publish_twist({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  }

  void publish_twist(const std::array<double, 6> & command)
  {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = world_frame_;
    msg.twist.linear.x = command[0];
    msg.twist.linear.y = command[1];
    msg.twist.linear.z = command[2];
    msg.twist.angular.x = command[3];
    msg.twist.angular.y = command[4];
    msg.twist.angular.z = command[5];
    twist_pub_->publish(msg);
  }

  std::string robot_name_;
  std::string demo_name_;
  std::string world_frame_;
  std::string twist_topic_;
  std::string object_pose_topic_;
  double xy_amplitude_{0.05};
  double z_lift_{0.05};
  double yaw_amplitude_deg_{5.0};
  double linear_velocity_{0.02};
  double angular_velocity_{0.10};
  int repetitions_{1};
  double publish_rate_hz_{500.0};
  double pose_timeout_{3.0};
  double min_segment_duration_{1.0};
  bool have_object_pose_{false};
  rclcpp::Time last_pose_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
};

}  // namespace match_cooperative_handling

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<match_cooperative_handling::VirtualObjectDemoRunner>();
  const int result = node->run();
  rclcpp::shutdown();
  return result;
}

#include "arena_mpc_controller/safety_aware_progress_checker.hpp"

#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>

#include <nav_2d_utils/conversions.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace arena_mpc_controller
{

void SafetyAwareProgressChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("SafetyAwareProgressChecker parent node expired");
  }
  clock_ = node->get_clock();

  const auto declare_double = [&node, &plugin_name](
    const std::string & name, const double value)
    {
      const std::string full_name = plugin_name + "." + name;
      if (!node->has_parameter(full_name)) {
        node->declare_parameter(full_name, value);
      }
      return node->get_parameter(full_name).as_double();
    };
  required_movement_radius_ = declare_double("required_movement_radius", 0.05);
  const double allowance = declare_double("movement_time_allowance", 120.0);
  const double status_timeout = declare_double("status_timeout", 1.0);
  if (!std::isfinite(required_movement_radius_) || required_movement_radius_ <= 0.0 ||
    !std::isfinite(allowance) || allowance <= 0.0 ||
    !std::isfinite(status_timeout) || status_timeout <= 0.0)
  {
    throw std::runtime_error("SafetyAwareProgressChecker parameters must be finite and positive");
  }
  time_allowance_ = rclcpp::Duration::from_seconds(allowance);
  status_timeout_ = rclcpp::Duration::from_seconds(status_timeout);

  const std::string topic_parameter = plugin_name + ".status_topic";
  if (!node->has_parameter(topic_parameter)) {
    node->declare_parameter(topic_parameter, std::string("/FollowPath/status"));
  }
  const std::string status_topic = node->get_parameter(topic_parameter).as_string();
  if (status_topic.empty()) {
    throw std::runtime_error("SafetyAwareProgressChecker status_topic must not be empty");
  }
  status_subscription_ = node->create_subscription<std_msgs::msg::String>(
    status_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
    std::bind(&SafetyAwareProgressChecker::on_status, this, std::placeholders::_1));
  reset();
}

bool SafetyAwareProgressChecker::check(geometry_msgs::msg::PoseStamped & current_pose)
{
  const auto pose = nav_2d_utils::poseToPose2D(current_pose.pose);
  const rclcpp::Time now = clock_->now();
  std::lock_guard<std::mutex> lock(mutex_);
  if (!baseline_pose_set_ || now < last_check_time_) {
    reset_baseline(pose, now);
    return true;
  }
  if (pose_distance(pose, baseline_pose_) > required_movement_radius_) {
    reset_baseline(pose, now);
    return true;
  }

  const rclcpp::Duration elapsed = now - last_check_time_;
  last_check_time_ = now;
  const bool status_fresh = status_received_ && now >= last_status_time_ &&
    (now - last_status_time_) <= status_timeout_;
  if (!(status_fresh && safety_wait_active_)) {
    active_tracking_time_ = active_tracking_time_ + elapsed;
  }
  return active_tracking_time_ <= time_allowance_;
}

void SafetyAwareProgressChecker::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  baseline_pose_set_ = false;
  status_received_ = false;
  safety_wait_active_ = false;
  active_tracking_time_ = rclcpp::Duration(0, 0);
}

bool SafetyAwareProgressChecker::is_safety_wait_status(const std::string & status)
{
  return status.rfind("stop recoverable=1 mode=safety_wait", 0U) == 0U ||
         (status.rfind("ok ", 0U) == 0U && status.find(" mode=human_wait") != std::string::npos);
}

double SafetyAwareProgressChecker::pose_distance(
  const geometry_msgs::msg::Pose2D & first,
  const geometry_msgs::msg::Pose2D & second)
{
  return std::hypot(first.x - second.x, first.y - second.y);
}

void SafetyAwareProgressChecker::reset_baseline(
  const geometry_msgs::msg::Pose2D & pose, const rclcpp::Time & now)
{
  baseline_pose_ = pose;
  last_check_time_ = now;
  active_tracking_time_ = rclcpp::Duration(0, 0);
  baseline_pose_set_ = true;
}

void SafetyAwareProgressChecker::on_status(const std_msgs::msg::String::SharedPtr message)
{
  const rclcpp::Time now = clock_->now();
  std::lock_guard<std::mutex> lock(mutex_);
  status_received_ = true;
  last_status_time_ = now;
  safety_wait_active_ = is_safety_wait_status(message->data);
}

}  // namespace arena_mpc_controller

PLUGINLIB_EXPORT_CLASS(
  arena_mpc_controller::SafetyAwareProgressChecker, nav2_core::ProgressChecker)

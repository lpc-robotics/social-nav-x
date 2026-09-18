#ifndef ARENA_MPC_CONTROLLER__SAFETY_AWARE_PROGRESS_CHECKER_HPP_
#define ARENA_MPC_CONTROLLER__SAFETY_AWARE_PROGRESS_CHECKER_HPP_

#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_core/progress_checker.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/string.hpp>

namespace arena_mpc_controller
{

struct SafetyAwareProgressCheckerTestAccess;

class SafetyAwareProgressChecker : public nav2_core::ProgressChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name) override;
  bool check(geometry_msgs::msg::PoseStamped & current_pose) override;
  void reset() override;

private:
  friend struct SafetyAwareProgressCheckerTestAccess;
  static bool is_safety_wait_status(const std::string & status);
  static double pose_distance(
    const geometry_msgs::msg::Pose2D & first,
    const geometry_msgs::msg::Pose2D & second);
  void reset_baseline(
    const geometry_msgs::msg::Pose2D & pose, const rclcpp::Time & now);
  void on_status(const std_msgs::msg::String::SharedPtr message);

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscription_;
  std::mutex mutex_;
  geometry_msgs::msg::Pose2D baseline_pose_;
  rclcpp::Time last_check_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_status_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Duration active_tracking_time_{0, 0};
  rclcpp::Duration time_allowance_{0, 0};
  rclcpp::Duration status_timeout_{0, 0};
  double required_movement_radius_{0.05};
  bool baseline_pose_set_{false};
  bool status_received_{false};
  bool safety_wait_active_{false};
};

}  // namespace arena_mpc_controller

#endif  // ARENA_MPC_CONTROLLER__SAFETY_AWARE_PROGRESS_CHECKER_HPP_

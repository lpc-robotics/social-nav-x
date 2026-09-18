#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "arena_mpc_controller/safety_aware_progress_checker.hpp"

using namespace std::chrono_literals;

namespace arena_mpc_controller
{
struct SafetyAwareProgressCheckerTestAccess
{
  static void set_status(SafetyAwareProgressChecker & checker, const std::string & value)
  {
    auto message = std::make_shared<std_msgs::msg::String>();
    message->data = value;
    checker.on_status(message);
  }
};
}  // namespace arena_mpc_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("progress_checker_test");
  node->declare_parameter("progress_checker.required_movement_radius", 0.05);
  node->declare_parameter("progress_checker.movement_time_allowance", 0.05);
  node->declare_parameter("progress_checker.status_timeout", 1.0);
  node->declare_parameter("progress_checker.status_topic", "/progress_checker_test/status");

  arena_mpc_controller::SafetyAwareProgressChecker checker;
  checker.initialize(node, "progress_checker");
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.orientation.w = 1.0;
  if (!checker.check(pose)) {
    std::cerr << "initial progress check failed" << std::endl;
    return 1;
  }
  std::this_thread::sleep_for(80ms);
  if (checker.check(pose)) {
    std::cerr << "ordinary tracking stall did not exhaust its allowance" << std::endl;
    return 2;
  }
  pose.pose.orientation.z = std::sin(0.11 / 2.0);
  pose.pose.orientation.w = std::cos(0.11 / 2.0);
  if (!checker.check(pose)) {
    std::cerr << "terminal yaw motion was not counted as progress" << std::endl;
    return 3;
  }

  checker.reset();
  if (!checker.check(pose)) {
    std::cerr << "progress reset failed" << std::endl;
    return 4;
  }
  arena_mpc_controller::SafetyAwareProgressCheckerTestAccess::set_status(
    checker, "stop recoverable=1 mode=safety_wait reason=test");
  std::this_thread::sleep_for(80ms);
  if (!checker.check(pose)) {
    std::cerr << "fresh safety wait consumed the active tracking allowance" << std::endl;
    return 5;
  }

  arena_mpc_controller::SafetyAwareProgressCheckerTestAccess::set_status(
    checker, "ok generation=1 epoch=1 solve_ms=1 cycle_ms=1 humans=1 mode=track");
  std::this_thread::sleep_for(80ms);
  if (checker.check(pose)) {
    std::cerr << "track status did not resume the active tracking allowance" << std::endl;
    return 6;
  }

  rclcpp::shutdown();
  std::cout << "SAFETY_AWARE_PROGRESS_CHECKER_OK" << std::endl;
  return 0;
}

#ifndef ARENA_MPC_CONTROLLER__MPC_CONTROLLER_HPP_
#define ARENA_MPC_CONTROLLER__MPC_CONTROLLER_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <arena_mpc_core/solver.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <hunav_msgs/msg/agents.hpp>
#include <nav2_core/controller.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>

namespace arena_mpc_controller
{

class MpcController : public nav2_core::Controller
{
public:
  MpcController() = default;
  ~MpcController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  using SteadyClock = std::chrono::steady_clock;

  struct InputState
  {
    hunav_msgs::msg::Agents humans;
    nav_msgs::msg::Odometry odom;
    builtin_interfaces::msg::Time lidar_stamp;
    SteadyClock::time_point human_wall_time{};
    SteadyClock::time_point odom_wall_time{};
    SteadyClock::time_point lidar_wall_time{};
    bool have_humans{false};
    bool have_odom{false};
    bool have_lidar{false};
    std::uint64_t human_sequence{0U};
    std::uint64_t odom_sequence{0U};
    std::uint64_t lidar_sequence{0U};
  };

  geometry_msgs::msg::TwistStamped fail(
    const std::string & reason, const std_msgs::msg::Header & header);
  geometry_msgs::msg::TwistStamped recoverable_stop(
    const std::string & reason, const std_msgs::msg::Header & header);
  geometry_msgs::msg::TwistStamped retry_stale_path(
    const std::string & reason, const std_msgs::msg::Header & header);
  void publish_status(const std::string & text);
  bool input_stamp_fresh(const builtin_interfaces::msg::Time & stamp) const;
  void on_humans(const hunav_msgs::msg::Agents::SharedPtr message);
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr message);
  void on_lidar(const sensor_msgs::msg::LaserScan::SharedPtr message);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<arena_mpc_core::Solver> solver_;
  arena_mpc_core::Config config_;

  rclcpp::Subscription<hunav_msgs::msg::Agents>::SharedPtr humans_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr trajectory_publisher_;

  std::mutex input_mutex_;
  InputState inputs_;
  std::mutex plan_mutex_;
  nav_msgs::msg::Path plan_;
  std::uint64_t plan_hash_{0U};
  std::atomic<std::uint64_t> path_generation_{0U};
  std::atomic<std::uint64_t> reset_epoch_{0U};
  std::atomic<double> speed_limit_{0.8};
  std::atomic<int> consecutive_failures_{0};
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  std::mutex solver_mutex_;
  std::uint64_t solver_epoch_{0U};

  double transform_tolerance_{0.2};
  double ros_age_limit_{0.3};
  double human_wall_limit_{0.60};
  double odom_wall_limit_{0.40};
  double lidar_wall_limit_{1.55};
  double plugin_commit_limit_ms_{90.0};
  double costmap_obstacle_wait_limit_{1.0};
  double reference_spacing_{0.025};
  double geometry_uncertainty_{0.05};
  double emergency_safe_distance_{0.30};
  double robot_circumscribed_radius_{0.326};
  int failure_limit_{5};
  std::optional<SteadyClock::time_point> costmap_wait_started_;
  bool active_{false};
};

}  // namespace arena_mpc_controller

#endif  // ARENA_MPC_CONTROLLER__MPC_CONTROLLER_HPP_

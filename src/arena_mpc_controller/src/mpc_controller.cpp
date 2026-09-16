#include "arena_mpc_controller/mpc_controller.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_set>
#include <utility>
#include <vector>

#include <arena_mpc_core/model.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav2_core/exceptions.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_filters/filter_values.hpp>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace arena_mpc_controller
{
namespace
{

constexpr double kBrakeDt = 0.05;

template<typename T>
T parameter(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name,
  const T & default_value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<T>(name, default_value);
  }
  T value{};
  if (!node->get_parameter(name, value)) {
    throw std::runtime_error("failed to read parameter " + name);
  }
  return value;
}

std::int64_t stamp_nanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

bool valid_stamp(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec != 0 || stamp.nanosec != 0U;
}

double point_distance(
  const geometry_msgs::msg::Point & first, const geometry_msgs::msg::Point & second)
{
  return std::hypot(first.x - second.x, first.y - second.y);
}

void hash_bytes(std::uint64_t & hash, const void * data, std::size_t size)
{
  const auto * bytes = static_cast<const unsigned char *>(data);
  for (std::size_t index = 0; index < size; ++index) {
    hash ^= bytes[index];
    hash *= 1099511628211ULL;
  }
}

std::uint64_t path_hash(const nav_msgs::msg::Path & path)
{
  std::uint64_t hash = 1469598103934665603ULL;
  hash_bytes(hash, path.header.frame_id.data(), path.header.frame_id.size());
  for (const auto & pose : path.poses) {
    const double values[] = {
      pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
      pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z,
      pose.pose.orientation.w};
    hash_bytes(hash, values, sizeof(values));
  }
  return hash;
}

geometry_msgs::msg::TwistStamped zero_command(const std_msgs::msg::Header & header)
{
  geometry_msgs::msg::TwistStamped command;
  command.header = header;
  return command;
}

std::vector<geometry_msgs::msg::Point> oriented_footprint(
  const std::vector<geometry_msgs::msg::Point> & footprint,
  const arena_mpc_core::State & state)
{
  std::vector<geometry_msgs::msg::Point> transformed;
  transformed.reserve(footprint.size());
  const double cosine = std::cos(state.yaw);
  const double sine = std::sin(state.yaw);
  for (const auto & point : footprint) {
    geometry_msgs::msg::Point output;
    output.x = state.x + cosine * point.x - sine * point.y;
    output.y = state.y + sine * point.x + cosine * point.y;
    transformed.push_back(output);
  }
  return transformed;
}

bool pose_collision_free(
  nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  const arena_mpc_core::State & state)
{
  const auto world_polygon = oriented_footprint(footprint, state);
  std::vector<nav2_costmap_2d::MapLocation> map_polygon;
  map_polygon.reserve(world_polygon.size());
  for (const auto & point : world_polygon) {
    unsigned int map_x = 0U;
    unsigned int map_y = 0U;
    if (!costmap.worldToMap(point.x, point.y, map_x, map_y)) {
      return false;
    }
    map_polygon.push_back(nav2_costmap_2d::MapLocation{map_x, map_y});
  }
  std::vector<nav2_costmap_2d::MapLocation> cells;
  costmap.convexFillCells(map_polygon, cells);
  if (cells.empty()) {
    return false;
  }
  for (const auto & cell : cells) {
    const unsigned char cost = costmap.getCost(cell.x, cell.y);
    if (cost == nav2_costmap_2d::NO_INFORMATION || cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
      return false;
    }
  }
  return true;
}

bool swept_trajectory_collision_free(
  nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  const std::vector<arena_mpc_core::State> & states, double circumscribed_radius,
  arena_mpc_core::State * collision_state = nullptr)
{
  if (states.empty()) {
    return false;
  }
  const double sample_motion = std::max(0.005, 0.5 * costmap.getResolution());
  if (!pose_collision_free(costmap, footprint, states.front())) {
    if (collision_state != nullptr) {
      *collision_state = states.front();
    }
    return false;
  }
  for (std::size_t index = 1U; index < states.size(); ++index) {
    const auto & previous = states[index - 1U];
    const auto & next = states[index];
    const double translation = std::hypot(next.x - previous.x, next.y - previous.y);
    const double yaw_delta = arena_mpc_core::wrap_angle(next.yaw - previous.yaw);
    const double corner_motion = std::abs(yaw_delta) * circumscribed_radius;
    const std::size_t samples = std::max<std::size_t>(
      1U, static_cast<std::size_t>(
        std::ceil(std::max(translation, corner_motion) / sample_motion)));
    for (std::size_t sample = 1U; sample <= samples; ++sample) {
      const double ratio = static_cast<double>(sample) / static_cast<double>(samples);
      const arena_mpc_core::State interpolated{
        previous.x + ratio * (next.x - previous.x),
        previous.y + ratio * (next.y - previous.y),
        previous.yaw + ratio * yaw_delta};
      if (!pose_collision_free(costmap, footprint, interpolated)) {
        if (collision_state != nullptr) {
          *collision_state = interpolated;
        }
        return false;
      }
    }
  }
  return true;
}

std::vector<arena_mpc_core::State> braking_trajectory(
  const arena_mpc_core::State & initial, const arena_mpc_core::Control & command,
  double linear_deceleration, double angular_deceleration)
{
  const auto approach_zero = [](double value, double step) {
      if (value > 0.0) {
        return std::max(0.0, value - step);
      }
      return std::min(0.0, value + step);
    };
  std::vector<arena_mpc_core::State> states{initial};
  arena_mpc_core::Control control = command;
  for (std::size_t step_index = 0U; step_index < 100U; ++step_index) {
    if (std::abs(control.linear) < 1.0e-6 && std::abs(control.angular) < 1.0e-6) {
      break;
    }
    states.push_back(arena_mpc_core::step(states.back(), control, kBrakeDt));
    control.linear = approach_zero(control.linear, linear_deceleration * kBrakeDt);
    control.angular = approach_zero(control.angular, angular_deceleration * kBrakeDt);
  }
  return states;
}

arena_mpc_core::Ellipse obstacle_at(
  const arena_mpc_core::ObstaclePrediction & obstacle, double time, double sample_dt)
{
  if (obstacle.samples.empty()) {
    return {};
  }
  if (time <= 0.0) {
    return obstacle.samples.front();
  }
  const double sample_position = time / sample_dt;
  const std::size_t lower = std::min<std::size_t>(
    static_cast<std::size_t>(std::floor(sample_position)), obstacle.samples.size() - 1U);
  const std::size_t upper = std::min(lower + 1U, obstacle.samples.size() - 1U);
  const double ratio = upper == lower ? 0.0 : sample_position - std::floor(sample_position);
  const auto & first = obstacle.samples[lower];
  const auto & second = obstacle.samples[upper];
  return {
    first.x + ratio * (second.x - first.x),
    first.y + ratio * (second.y - first.y),
    first.semi_major + ratio * (second.semi_major - first.semi_major),
    first.semi_minor + ratio * (second.semi_minor - first.semi_minor),
    first.yaw + ratio * arena_mpc_core::wrap_angle(second.yaw - first.yaw)};
}

bool costmap_collision_matches_human(
  const arena_mpc_core::State & collision_state,
  const std::vector<arena_mpc_core::ObstaclePrediction> & obstacles,
  double sample_dt, double safe_distance, double clearing_time)
{
  for (const auto & obstacle : obstacles) {
    if (!obstacle.dynamic || obstacle.samples.empty()) {
      continue;
    }
    const auto & current = obstacle.samples.front();
    double speed = 0.0;
    if (obstacle.samples.size() >= 2U && sample_dt > 0.0) {
      speed = std::hypot(
        obstacle.samples[1U].x - current.x,
        obstacle.samples[1U].y - current.y) / sample_dt;
    }
    const double association_radius = std::max(current.semi_major, current.semi_minor) +
      safe_distance + speed * clearing_time;
    if (std::hypot(collision_state.x - current.x, collision_state.y - current.y) <=
      association_radius)
    {
      return true;
    }
  }
  return false;
}

bool dynamic_trajectory_collision_free(
  const std::vector<arena_mpc_core::State> & states, double state_dt,
  const std::vector<arena_mpc_core::ObstaclePrediction> & obstacles,
  double obstacle_dt, double safe_distance)
{
  if (states.empty() || !(state_dt > 0.0) || !(obstacle_dt > 0.0)) {
    return false;
  }
  for (const auto & obstacle : obstacles) {
    if (!obstacle.dynamic || obstacle.samples.empty()) {
      continue;
    }
    if (states.size() == 1U) {
      const auto sample = obstacle_at(obstacle, 0.0, obstacle_dt);
      if (std::hypot(states.front().x - sample.x, states.front().y - sample.y) <
        std::max(sample.semi_major, sample.semi_minor) + safe_distance)
      {
        return false;
      }
      continue;
    }
    for (std::size_t index = 1U; index < states.size(); ++index) {
      const double first_time = state_dt * static_cast<double>(index - 1U);
      const double second_time = state_dt * static_cast<double>(index);
      const auto first_obstacle = obstacle_at(obstacle, first_time, obstacle_dt);
      const auto second_obstacle = obstacle_at(obstacle, second_time, obstacle_dt);
      const double relative_x = states[index - 1U].x - first_obstacle.x;
      const double relative_y = states[index - 1U].y - first_obstacle.y;
      const double relative_delta_x =
        (states[index].x - second_obstacle.x) - relative_x;
      const double relative_delta_y =
        (states[index].y - second_obstacle.y) - relative_y;
      const double relative_motion_squared =
        relative_delta_x * relative_delta_x + relative_delta_y * relative_delta_y;
      const double closest_ratio = relative_motion_squared > 1.0e-12 ?
        std::clamp(
          -(relative_x * relative_delta_x + relative_y * relative_delta_y) /
          relative_motion_squared,
          0.0, 1.0) : 0.0;
      const double closest_distance = std::hypot(
        relative_x + closest_ratio * relative_delta_x,
        relative_y + closest_ratio * relative_delta_y);
      const double required_distance = safe_distance + std::max({
          first_obstacle.semi_major, first_obstacle.semi_minor,
          second_obstacle.semi_major, second_obstacle.semi_minor});
      if (closest_distance + 1.0e-9 < required_distance) {
        return false;
      }
    }
  }
  return true;
}

double point_to_polygon_signed_distance(
  double x, double y, const std::vector<geometry_msgs::msg::Point> & polygon)
{
  if (polygon.size() < 3U) {
    return -std::numeric_limits<double>::infinity();
  }
  bool inside = false;
  double minimum_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < polygon.size(); ++index) {
    const auto & first = polygon[index];
    const auto & second = polygon[(index + 1U) % polygon.size()];
    const double edge_x = second.x - first.x;
    const double edge_y = second.y - first.y;
    const double edge_length_squared = edge_x * edge_x + edge_y * edge_y;
    const double ratio = edge_length_squared > 1.0e-18 ?
      std::clamp(
      ((x - first.x) * edge_x + (y - first.y) * edge_y) / edge_length_squared,
      0.0, 1.0) : 0.0;
    minimum_distance = std::min(
      minimum_distance,
      std::hypot(x - (first.x + ratio * edge_x), y - (first.y + ratio * edge_y)));
    if ((first.y > y) != (second.y > y)) {
      const double crossing_x = first.x + (y - first.y) * edge_x / edge_y;
      if (x < crossing_x) {
        inside = !inside;
      }
    }
  }
  return inside ? -minimum_distance : minimum_distance;
}

double footprint_circle_clearance(
  const arena_mpc_core::State & state,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  const arena_mpc_core::Ellipse & obstacle, double circumscribed_radius)
{
  const double cosine = std::cos(state.yaw);
  const double sine = std::sin(state.yaw);
  const double dx = obstacle.x - state.x;
  const double dy = obstacle.y - state.y;
  const double local_x = cosine * dx + sine * dy;
  const double local_y = -sine * dx + cosine * dy;
  const double circle_radius =
    std::max(obstacle.semi_major, obstacle.semi_minor) - circumscribed_radius;
  if (!(circle_radius > 0.0) || !std::isfinite(circle_radius)) {
    return -std::numeric_limits<double>::infinity();
  }
  return point_to_polygon_signed_distance(local_x, local_y, footprint) - circle_radius;
}

// The NLP uses a circumscribed robot circle.  During an emergency stop that
// approximation can report a collision even though the configured polygon can
// still brake safely.  Check the measured braking motion against the polygon,
// and subtract a Lipschitz bound for motion between samples so the test remains
// conservative over the complete swept interval.
bool measured_braking_dynamic_collision_free(
  const std::vector<arena_mpc_core::State> & states, double state_dt,
  const std::vector<arena_mpc_core::ObstaclePrediction> & obstacles,
  double obstacle_dt, const std::vector<geometry_msgs::msg::Point> & footprint,
  double circumscribed_radius, double safe_distance)
{
  if (states.empty() || !(state_dt > 0.0) || !(obstacle_dt > 0.0) ||
    footprint.size() < 3U)
  {
    return false;
  }
  for (const auto & obstacle : obstacles) {
    if (!obstacle.dynamic || obstacle.samples.empty()) {
      continue;
    }
    auto previous_obstacle = obstacle_at(obstacle, 0.0, obstacle_dt);
    double previous_clearance = footprint_circle_clearance(
      states.front(), footprint, previous_obstacle, circumscribed_radius);
    if (!std::isfinite(previous_clearance) || previous_clearance < safe_distance) {
      return false;
    }
    for (std::size_t index = 1U; index < states.size(); ++index) {
      const auto next_obstacle = obstacle_at(
        obstacle, state_dt * static_cast<double>(index), obstacle_dt);
      const double next_clearance = footprint_circle_clearance(
        states[index], footprint, next_obstacle, circumscribed_radius);
      const double robot_translation = std::hypot(
        states[index].x - states[index - 1U].x,
        states[index].y - states[index - 1U].y);
      const double robot_rotation = circumscribed_radius * std::abs(
        arena_mpc_core::wrap_angle(states[index].yaw - states[index - 1U].yaw));
      const double obstacle_translation = std::hypot(
        next_obstacle.x - previous_obstacle.x,
        next_obstacle.y - previous_obstacle.y);
      const double obstacle_radius_change = std::abs(
        std::max(next_obstacle.semi_major, next_obstacle.semi_minor) -
        std::max(previous_obstacle.semi_major, previous_obstacle.semi_minor));
      const double between_sample_bound = 0.5 * (
        robot_translation + robot_rotation + obstacle_translation + obstacle_radius_change);
      if (!std::isfinite(next_clearance) ||
        std::min(previous_clearance, next_clearance) - between_sample_bound < safe_distance)
      {
        return false;
      }
      previous_obstacle = next_obstacle;
      previous_clearance = next_clearance;
    }
  }
  return true;
}

}  // namespace

void MpcController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error("MpcController parent lifecycle node expired");
  }
  plugin_name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  const std::string prefix = plugin_name_ + ".";

  config_.horizon = static_cast<std::size_t>(parameter<std::int64_t>(node_, prefix + "horizon", 25));
  config_.dt = parameter<double>(node_, prefix + "dt", 0.1);
  config_.gamma = parameter<double>(node_, prefix + "gamma", 0.2);
  config_.safe_distance = parameter<double>(node_, prefix + "safe_distance", 0.3);
  config_.terminal_weight = parameter<double>(node_, prefix + "terminal_weight", 1.0);
  config_.slack_weight = parameter<double>(node_, prefix + "slack_weight", 50.0);
  config_.max_linear = parameter<double>(node_, prefix + "max_linear", 0.8);
  config_.max_angular = parameter<double>(node_, prefix + "max_angular", 1.5);
  config_.max_linear_accel = parameter<double>(node_, prefix + "max_linear_accel", 2.0);
  config_.max_angular_accel = parameter<double>(node_, prefix + "max_angular_accel", 3.2);
  config_.max_dynamic_obstacles = static_cast<std::size_t>(
    parameter<std::int64_t>(node_, prefix + "max_dynamic_obstacles", 32));
  config_.max_static_obstacles = 128U;
  config_.max_nlp_dynamic_obstacles = static_cast<std::size_t>(
    parameter<std::int64_t>(node_, prefix + "max_nlp_dynamic_obstacles", 8));
  config_.max_nlp_static_obstacles = 0U;
  config_.max_iterations = static_cast<int>(
    parameter<std::int64_t>(node_, prefix + "max_iterations", 100));
  config_.solver_budget_seconds = 0.001 *
    parameter<double>(node_, prefix + "solver_budget_ms", 75.0);
  config_.layout = arena_mpc_core::SolverLayout::FixedMasked;

  transform_tolerance_ = parameter<double>(node_, prefix + "transform_tolerance", 0.2);
  ros_age_limit_ = parameter<double>(node_, prefix + "ros_age_limit", 0.3);
  human_wall_limit_ = parameter<double>(node_, prefix + "human_wall_limit", 0.60);
  odom_wall_limit_ = parameter<double>(node_, prefix + "odom_wall_limit", 0.40);
  lidar_wall_limit_ = parameter<double>(node_, prefix + "lidar_wall_limit", 1.55);
  plugin_commit_limit_ms_ = parameter<double>(node_, prefix + "plugin_commit_limit_ms", 90.0);
  costmap_obstacle_wait_limit_ = parameter<double>(
    node_, prefix + "costmap_obstacle_wait_limit", 1.0);
  reference_spacing_ = parameter<double>(node_, prefix + "reference_spacing", 0.025);
  geometry_uncertainty_ = parameter<double>(node_, prefix + "geometry_uncertainty", 0.05);
  emergency_safe_distance_ = parameter<double>(
    node_, prefix + "emergency_safe_distance", 0.30);
  if (emergency_safe_distance_ < 0.0 || emergency_safe_distance_ > config_.safe_distance) {
    throw std::runtime_error(
            "emergency_safe_distance must be non-negative and no greater than safe_distance");
  }
  failure_limit_ = static_cast<int>(parameter<std::int64_t>(node_, prefix + "failure_limit", 5));
  const std::string humans_topic = parameter<std::string>(node_, prefix + "humans_topic", "/human_states");
  const std::string odom_topic = parameter<std::string>(node_, prefix + "odom_topic", "/odom");
  const std::string lidar_topic = parameter<std::string>(node_, prefix + "lidar_topic", "/lidar");

  const auto footprint = costmap_ros_->getRobotFootprint();
  if (footprint.size() < 3U) {
    throw std::runtime_error("MPC requires a polygon footprint with at least three points");
  }
  robot_circumscribed_radius_ = 0.0;
  for (const auto & point : footprint) {
    robot_circumscribed_radius_ = std::max(
      robot_circumscribed_radius_, std::hypot(point.x, point.y));
  }
  if (std::abs(robot_circumscribed_radius_ - std::hypot(0.24, 0.22)) > 0.02) {
    throw std::runtime_error("costmap footprint does not match the MPC Jackal geometry");
  }

  speed_limit_.store(config_.max_linear);
  solver_ = std::make_unique<arena_mpc_core::Solver>(config_);
  status_publisher_ = node_->create_publisher<std_msgs::msg::String>(
    plugin_name_ + "/status", rclcpp::QoS(10));
  trajectory_publisher_ = node_->create_publisher<nav_msgs::msg::Path>(
    plugin_name_ + "/predicted_path", rclcpp::QoS(1));
  humans_subscription_ = node_->create_subscription<hunav_msgs::msg::Agents>(
    humans_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
    std::bind(&MpcController::on_humans, this, std::placeholders::_1));
  odom_subscription_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
    std::bind(&MpcController::on_odom, this, std::placeholders::_1));
  lidar_subscription_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    lidar_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
    std::bind(&MpcController::on_lidar, this, std::placeholders::_1));

  reset_epoch_.fetch_add(1U);
  consecutive_failures_.store(0);
  RCLCPP_INFO(
    node_->get_logger(),
    "Configured %s: CasADi graph N=%zu dynamic_slots=%zu build occurs outside control loop",
    plugin_name_.c_str(), config_.horizon, config_.max_nlp_dynamic_obstacles);
}

void MpcController::cleanup()
{
  active_ = false;
  reset_epoch_.fetch_add(1U);
  solver_.reset();
  humans_subscription_.reset();
  odom_subscription_.reset();
  lidar_subscription_.reset();
  trajectory_publisher_.reset();
  status_publisher_.reset();
  costmap_ros_.reset();
  tf_.reset();
  node_.reset();
}

void MpcController::activate()
{
  status_publisher_->on_activate();
  trajectory_publisher_->on_activate();
  active_ = true;
  reset_epoch_.fetch_add(1U);
  solver_->reset();
  consecutive_failures_.store(0);
  costmap_wait_started_.reset();
  publish_status("active; waiting for fresh path, odom, HuNav, lidar, and costmap");
}

void MpcController::deactivate()
{
  active_ = false;
  reset_epoch_.fetch_add(1U);
  solver_->reset();
  consecutive_failures_.store(0);
  costmap_wait_started_.reset();
  trajectory_publisher_->on_deactivate();
  status_publisher_->on_deactivate();
}

void MpcController::setPlan(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty() || path.header.frame_id.empty()) {
    throw nav2_core::PlannerException("MPC rejected an empty or frameless global path");
  }
  const std::uint64_t hash = path_hash(path);
  std::lock_guard<std::mutex> lock(plan_mutex_);
  plan_ = path;
  if (hash != plan_hash_) {
    plan_hash_ = hash;
    path_generation_.fetch_add(1U);
  }
}

geometry_msgs::msg::TwistStamped MpcController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  (void)velocity;
  (void)goal_checker;
  const auto cycle_start = SteadyClock::now();
  std_msgs::msg::Header output_header = pose.header;
  output_header.stamp = node_->now();
  if (!active_) {
    return fail("controller is inactive", output_header);
  }
  if (!costmap_ros_->isCurrent()) {
    return fail("local costmap is not current", output_header);
  }

  InputState inputs;
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    inputs = inputs_;
  }
  nav_msgs::msg::Path plan;
  {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    plan = plan_;
  }
  const std::uint64_t epoch = reset_epoch_.load();
  const std::uint64_t generation = path_generation_.load();
  {
    std::lock_guard<std::mutex> lock(solver_mutex_);
    if (solver_epoch_ != epoch) {
      solver_->reset();
      solver_epoch_ = epoch;
      last_command_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
  }
  if (plan.poses.empty()) {
    return fail("no global path", output_header);
  }
  if (!inputs.have_humans || !inputs.have_odom || !inputs.have_lidar) {
    return fail("required input has not been received", output_header);
  }

  const auto wall_now = SteadyClock::now();
  const auto wall_age = [&](SteadyClock::time_point reception) {
      return std::chrono::duration<double>(wall_now - reception).count();
    };
  const double human_wall_age = wall_age(inputs.human_wall_time);
  const double odom_wall_age = wall_age(inputs.odom_wall_time);
  const double lidar_wall_age = wall_age(inputs.lidar_wall_time);
  if (human_wall_age > human_wall_limit_ || odom_wall_age > odom_wall_limit_ ||
    lidar_wall_age > lidar_wall_limit_)
  {
    std::ostringstream reason;
    reason << "required input exceeded wall-clock freshness limit"
           << " human_age=" << human_wall_age
           << " odom_age=" << odom_wall_age
           << " lidar_age=" << lidar_wall_age;
    return fail(reason.str(), output_header);
  }

  const rclcpp::Time now = node_->now();
  const auto ros_age = [&](const builtin_interfaces::msg::Time & stamp) {
      return (now - rclcpp::Time(stamp, RCL_ROS_TIME)).seconds();
    };
  const double human_age = ros_age(inputs.humans.header.stamp);
  const double odom_age = ros_age(inputs.odom.header.stamp);
  const double lidar_age = ros_age(inputs.lidar_stamp);
  if (!std::isfinite(human_age) || !std::isfinite(odom_age) || !std::isfinite(lidar_age) ||
    human_age < -0.05 || odom_age < -0.05 || lidar_age < -0.05 ||
    human_age > ros_age_limit_ || odom_age > ros_age_limit_ || lidar_age > ros_age_limit_)
  {
    return fail("required input exceeded ROS-time freshness limit", output_header);
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  if (!costmap_ros_->transformPoseToGlobalFrame(pose, robot_pose)) {
    return fail("robot pose transform failed", output_header);
  }
  const std::string target_frame = costmap_ros_->getGlobalFrameID();
  std::vector<geometry_msgs::msg::PoseStamped> transformed_plan;
  transformed_plan.reserve(plan.poses.size());
  try {
    for (auto plan_pose : plan.poses) {
      plan_pose.header.frame_id = plan.header.frame_id;
      plan_pose.header.stamp = pose.header.stamp;
      geometry_msgs::msg::PoseStamped transformed;
      tf_->transform(
        plan_pose, transformed, target_frame,
        tf2::durationFromSec(transform_tolerance_));
      transformed_plan.push_back(std::move(transformed));
    }
  } catch (const tf2::TransformException & error) {
    return fail(std::string("path transform failed: ") + error.what(), output_header);
  }

  std::size_t nearest = 0U;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < transformed_plan.size(); ++index) {
    const double distance = point_distance(
      robot_pose.pose.position, transformed_plan[index].pose.position);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest = index;
    }
  }
  std::vector<double> cumulative(transformed_plan.size() - nearest, 0.0);
  for (std::size_t index = 1U; index < cumulative.size(); ++index) {
    cumulative[index] = cumulative[index - 1U] + point_distance(
      transformed_plan[nearest + index - 1U].pose.position,
      transformed_plan[nearest + index].pose.position);
  }

  arena_mpc_core::Problem problem;
  problem.initial_state = {
    robot_pose.pose.position.x, robot_pose.pose.position.y,
    tf2::getYaw(robot_pose.pose.orientation)};
  problem.measured_control = {
    inputs.odom.twist.twist.linear.x, inputs.odom.twist.twist.angular.z};
  problem.linear_speed_limit = speed_limit_.load();
  problem.first_interval = config_.dt;
  if (last_command_time_.nanoseconds() != 0) {
    const double interval = (now - last_command_time_).seconds();
    if (interval < 0.0) {
      reset_epoch_.fetch_add(1U);
      return fail("simulation clock moved backwards", output_header);
    }
  }

  problem.reference.reserve(config_.horizon + 1U);
  for (std::size_t step_index = 0U; step_index <= config_.horizon; ++step_index) {
    const double wanted = reference_spacing_ * static_cast<double>(step_index);
    const auto upper = std::lower_bound(cumulative.begin(), cumulative.end(), wanted);
    std::size_t local_index = upper == cumulative.end() ? cumulative.size() - 1U :
      static_cast<std::size_t>(std::distance(cumulative.begin(), upper));
    arena_mpc_core::State reference;
    if (local_index == 0U || cumulative[local_index] <= wanted) {
      const auto & selected = transformed_plan[nearest + local_index].pose;
      reference.x = selected.position.x;
      reference.y = selected.position.y;
    } else {
      const double lower_distance = cumulative[local_index - 1U];
      const double segment = cumulative[local_index] - lower_distance;
      const double ratio = segment > 1.0e-9 ? (wanted - lower_distance) / segment : 0.0;
      const auto & first = transformed_plan[nearest + local_index - 1U].pose.position;
      const auto & second = transformed_plan[nearest + local_index].pose.position;
      reference.x = first.x + ratio * (second.x - first.x);
      reference.y = first.y + ratio * (second.y - first.y);
    }
    const std::size_t tangent_index = std::min(local_index + 1U, cumulative.size() - 1U);
    const auto & tangent_target = transformed_plan[nearest + tangent_index].pose.position;
    const double dx = tangent_target.x - reference.x;
    const double dy = tangent_target.y - reference.y;
    reference.yaw = std::hypot(dx, dy) > 1.0e-9 ?
      std::atan2(dy, dx) : tf2::getYaw(transformed_plan.back().pose.orientation);
    problem.reference.push_back(reference);
  }

  const auto build_human_obstacles = [this, &target_frame](
    const hunav_msgs::msg::Agents & humans, double age,
    std::vector<arena_mpc_core::ObstaclePrediction> & obstacles,
    std::string & error) -> bool
    {
      if (humans.agents.size() > config_.max_dynamic_obstacles) {
        error = "HuNav input capacity exceeded";
        return false;
      }
      try {
        const auto transform = tf_->lookupTransform(
          target_frame, humans.header.frame_id,
          rclcpp::Time(humans.header.stamp, RCL_ROS_TIME),
          tf2::durationFromSec(transform_tolerance_));
        const double frame_yaw = tf2::getYaw(transform.transform.rotation);
        const double cosine = std::cos(frame_yaw);
        const double sine = std::sin(frame_yaw);
        std::unordered_set<std::int32_t> human_ids;
        obstacles.clear();
        obstacles.reserve(humans.agents.size());
        for (const auto & human : humans.agents) {
          if (human.id < 0 || !std::isfinite(human.radius) || human.radius <= 0.0 ||
            !std::isfinite(human.velocity.linear.x) || !std::isfinite(human.velocity.linear.y))
          {
            error = "HuNav message contains invalid agent geometry";
            return false;
          }
          if (!human_ids.insert(human.id).second) {
            error = "HuNav message contains duplicate agent IDs";
            return false;
          }
          geometry_msgs::msg::PoseStamped source_pose;
          source_pose.header = humans.header;
          source_pose.pose = human.position;
          geometry_msgs::msg::PoseStamped transformed_pose;
          tf2::doTransform(source_pose, transformed_pose, transform);
          const double velocity_x =
            cosine * human.velocity.linear.x - sine * human.velocity.linear.y;
          const double velocity_y =
            sine * human.velocity.linear.x + cosine * human.velocity.linear.y;
          arena_mpc_core::ObstaclePrediction obstacle;
          obstacle.id = static_cast<std::uint64_t>(human.id);
          obstacle.dynamic = true;
          obstacle.samples.reserve(config_.horizon + 1U);
          const double expanded_radius = human.radius + robot_circumscribed_radius_ +
            geometry_uncertainty_;
          for (std::size_t step_index = 0U; step_index <= config_.horizon; ++step_index) {
            const double prediction_time = std::max(0.0, age) +
              config_.dt * static_cast<double>(step_index);
            obstacle.samples.push_back({
                transformed_pose.pose.position.x + prediction_time * velocity_x,
                transformed_pose.pose.position.y + prediction_time * velocity_y,
                expanded_radius, expanded_radius, 0.0});
          }
          obstacles.push_back(std::move(obstacle));
        }
        return true;
      } catch (const tf2::TransformException & transform_error) {
        error = std::string("HuNav transform failed: ") + transform_error.what();
        return false;
      }
    };

  std::string human_error;
  if (!build_human_obstacles(inputs.humans, human_age, problem.obstacles, human_error)) {
    return fail(human_error, output_header);
  }

  arena_mpc_core::Result result;
  {
    std::lock_guard<std::mutex> lock(solver_mutex_);
    result = solver_->solve(problem, true);
  }
  const bool solver_candidate_valid =
    result.command_valid && !result.trajectory.controls.empty();
  const bool solver_wait_allowed = !solver_candidate_valid &&
    (result.code == arena_mpc_core::SolveCode::Timeout ||
    result.code == arena_mpc_core::SolveCode::Infeasible);
  if (!solver_candidate_valid && !solver_wait_allowed) {
    std::ostringstream reason;
    reason << "MPC solve rejected: " << result.status
           << " solve_ms=" << result.timing.solve_ms;
    return fail(reason.str(), output_header);
  }

  if (epoch != reset_epoch_.load()) {
    return fail("solve result reset epoch became stale", output_header);
  }
  if (generation != path_generation_.load()) {
    return retry_stale_path("solve result path generation became stale", output_header);
  }

  std::vector<arena_mpc_core::State> braking;
  bool wait_for_human = solver_wait_allowed;
  if (solver_candidate_valid) {
    braking = braking_trajectory(
      problem.initial_state, result.trajectory.controls.front(),
      config_.max_linear_accel, config_.max_angular_accel);
    const bool candidate_dynamic_safe = dynamic_trajectory_collision_free(
        result.trajectory.states, config_.dt, problem.obstacles, config_.dt,
        config_.safe_distance) && dynamic_trajectory_collision_free(
        braking, kBrakeDt, problem.obstacles, config_.dt, config_.safe_distance);
    wait_for_human = !candidate_dynamic_safe;
  }
  const auto measured_braking = braking_trajectory(
    problem.initial_state, problem.measured_control,
    config_.max_linear_accel, config_.max_angular_accel);
  auto * costmap = costmap_ros_->getCostmap();
  const auto footprint = costmap_ros_->getRobotFootprint();
  if (wait_for_human) {
    if (!measured_braking_dynamic_collision_free(
        measured_braking, kBrakeDt, problem.obstacles, config_.dt, footprint,
        robot_circumscribed_radius_, emergency_safe_distance_))
    {
      std::ostringstream reason;
      reason << "measured braking trajectory failed swept HuNav check";
      if (solver_wait_allowed) {
        reason << " after MPC solve rejected: " << result.status;
      }
      return fail(reason.str(), output_header);
    }
  }

  bool measured_braking_static_safe = false;
  bool costmap_wait_allowed = false;
  {
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap->getMutex());
    arena_mpc_core::State collision_state;
    measured_braking_static_safe = swept_trajectory_collision_free(
      *costmap, footprint, measured_braking, robot_circumscribed_radius_, &collision_state);
    if (wait_for_human) {
      costmap_wait_started_.reset();
      if (!measured_braking_static_safe) {
        std::ostringstream reason;
        reason << "measured braking trajectory failed full-footprint costmap check at x="
               << collision_state.x << " y=" << collision_state.y
               << " yaw=" << collision_state.yaw;
        return fail(reason.str(), output_header);
      }
    } else {
      std::string candidate_collision_reason;
      if (!swept_trajectory_collision_free(
          *costmap, footprint, result.trajectory.states, robot_circumscribed_radius_,
          &collision_state))
      {
        std::ostringstream reason;
        reason << "predicted trajectory failed full-footprint costmap check at x="
               << collision_state.x << " y=" << collision_state.y
               << " yaw=" << collision_state.yaw;
        candidate_collision_reason = reason.str();
      }
      if (candidate_collision_reason.empty() && !swept_trajectory_collision_free(
          *costmap, footprint, braking, robot_circumscribed_radius_, &collision_state))
      {
        std::ostringstream reason;
        reason << "braking trajectory failed full-footprint costmap check at x="
               << collision_state.x << " y=" << collision_state.y
               << " yaw=" << collision_state.yaw;
        candidate_collision_reason = reason.str();
      }
      if (!candidate_collision_reason.empty()) {
        if (!measured_braking_static_safe) {
          return fail(candidate_collision_reason + "; measured braking is unsafe", output_header);
        }
        if (!costmap_collision_matches_human(
            collision_state, problem.obstacles, config_.dt, config_.safe_distance,
            costmap_obstacle_wait_limit_))
        {
          return fail(candidate_collision_reason, output_header);
        }
        if (!costmap_wait_started_) {
          costmap_wait_started_ = cycle_start;
        }
        const double wait_age = std::chrono::duration<double>(
          cycle_start - *costmap_wait_started_).count();
        if (wait_age > costmap_obstacle_wait_limit_) {
          std::ostringstream reason;
          reason << candidate_collision_reason
                 << "; persisted beyond costmap wait limit "
                 << costmap_obstacle_wait_limit_ << " s";
          return fail(reason.str(), output_header);
        }
        wait_for_human = true;
        costmap_wait_allowed = true;
      } else {
        costmap_wait_started_.reset();
      }
    }
  }

  if (epoch != reset_epoch_.load()) {
    return fail("reset epoch became stale during postcheck", output_header);
  }
  if (generation != path_generation_.load()) {
    return retry_stale_path("path generation became stale during postcheck", output_header);
  }
  const auto final_wall_now = SteadyClock::now();
  const rclcpp::Time final_ros_now = node_->now();
  InputState final_inputs;
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    final_inputs = inputs_;
    if (std::chrono::duration<double>(final_wall_now - final_inputs.human_wall_time).count() >
      human_wall_limit_ ||
      std::chrono::duration<double>(final_wall_now - final_inputs.odom_wall_time).count() >
      odom_wall_limit_ ||
      std::chrono::duration<double>(final_wall_now - final_inputs.lidar_wall_time).count() >
      lidar_wall_limit_)
    {
      std::ostringstream reason;
      reason << "input expired before command commit"
             << " human_age="
             << std::chrono::duration<double>(
        final_wall_now - final_inputs.human_wall_time).count()
             << " odom_age="
             << std::chrono::duration<double>(
        final_wall_now - final_inputs.odom_wall_time).count()
             << " lidar_age="
             << std::chrono::duration<double>(
        final_wall_now - final_inputs.lidar_wall_time).count();
      return fail(reason.str(), output_header);
    }
  }
  const auto final_age = [&final_ros_now](const builtin_interfaces::msg::Time & stamp) {
      return (final_ros_now - rclcpp::Time(stamp, RCL_ROS_TIME)).seconds();
    };
  const double final_human_age = final_age(final_inputs.humans.header.stamp);
  const double final_odom_age = final_age(final_inputs.odom.header.stamp);
  const double final_lidar_age = final_age(final_inputs.lidar_stamp);
  if (!std::isfinite(final_human_age) || !std::isfinite(final_odom_age) ||
    !std::isfinite(final_lidar_age) || final_human_age < -0.05 ||
    final_odom_age < -0.05 || final_lidar_age < -0.05 ||
    final_human_age > ros_age_limit_ || final_odom_age > ros_age_limit_ ||
    final_lidar_age > ros_age_limit_)
  {
    return fail("input ROS timestamp expired before command commit", output_header);
  }
  if (final_inputs.human_sequence != inputs.human_sequence) {
    std::vector<arena_mpc_core::ObstaclePrediction> latest_obstacles;
    if (!build_human_obstacles(
        final_inputs.humans, final_human_age, latest_obstacles, human_error))
    {
      return fail(human_error, output_header);
    }
    const bool latest_measured_braking_safe = measured_braking_dynamic_collision_free(
      measured_braking, kBrakeDt, latest_obstacles, config_.dt, footprint,
      robot_circumscribed_radius_, emergency_safe_distance_);
    const bool latest_candidate_safe = solver_candidate_valid &&
      dynamic_trajectory_collision_free(
      result.trajectory.states, config_.dt, latest_obstacles, config_.dt,
      config_.safe_distance) && dynamic_trajectory_collision_free(
      braking, kBrakeDt, latest_obstacles, config_.dt, config_.safe_distance);
    if (wait_for_human && !latest_measured_braking_safe) {
      return fail(
        "latest HuNav update invalidated measured braking trajectory", output_header);
    }
    if (!wait_for_human && !latest_candidate_safe) {
      if (!latest_measured_braking_safe || !measured_braking_static_safe) {
        return fail("latest HuNav update failed swept postcheck", output_header);
      }
      wait_for_human = true;
    }
    std::lock_guard<std::mutex> lock(input_mutex_);
    if (inputs_.human_sequence != final_inputs.human_sequence) {
      return fail("HuNav input changed during final postcheck", output_header);
    }
  }
  if (epoch != reset_epoch_.load()) {
    return fail("reset epoch became stale before command commit", output_header);
  }
  if (generation != path_generation_.load()) {
    return retry_stale_path("path generation became stale before command commit", output_header);
  }
  const double elapsed_ms = std::chrono::duration<double, std::milli>(
    SteadyClock::now() - cycle_start).count();
  if (elapsed_ms > plugin_commit_limit_ms_) {
    return fail("plugin commit deadline exceeded", output_header);
  }

  geometry_msgs::msg::TwistStamped command;
  command.header = output_header;
  if (!wait_for_human && solver_candidate_valid) {
    command.twist.linear.x = result.trajectory.controls.front().linear;
    command.twist.angular.z = result.trajectory.controls.front().angular;
  }
  if (!std::isfinite(command.twist.linear.x) || !std::isfinite(command.twist.angular.z)) {
    return fail("solver produced a non-finite command", output_header);
  }
  last_command_time_ = now;
  consecutive_failures_.store(0);

  nav_msgs::msg::Path predicted_path;
  predicted_path.header = output_header;
  predicted_path.header.frame_id = target_frame;
  const auto & published_states = wait_for_human ? measured_braking : result.trajectory.states;
  for (const auto & state : published_states) {
    geometry_msgs::msg::PoseStamped predicted;
    predicted.header = predicted_path.header;
    predicted.pose.position.x = state.x;
    predicted.pose.position.y = state.y;
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, state.yaw);
    predicted.pose.orientation = tf2::toMsg(quaternion);
    predicted_path.poses.push_back(std::move(predicted));
  }
  trajectory_publisher_->publish(predicted_path);
  std::ostringstream status;
  status << "ok generation=" << generation << " epoch=" << epoch
         << " solve_ms=" << result.timing.solve_ms << " cycle_ms=" << elapsed_ms
         << " humans=" << problem.obstacles.size()
         << " mode=" << (wait_for_human ? "human_wait" : "track");
  if (costmap_wait_allowed) {
    status << " wait_reason=costmap_postcheck";
  } else if (solver_wait_allowed) {
    status << " wait_reason=solver_"
           << (result.code == arena_mpc_core::SolveCode::Timeout ? "timeout" : "infeasible");
  } else if (wait_for_human) {
    status << " wait_reason=dynamic_postcheck";
  }
  publish_status(status.str());
  return command;
}

void MpcController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  double selected = config_.max_linear;
  if (speed_limit != nav2_costmap_2d::NO_SPEED_LIMIT) {
    selected = percentage ? config_.max_linear * speed_limit / 100.0 : speed_limit;
    selected = std::clamp(selected, 0.001, config_.max_linear);
  }
  speed_limit_.store(selected);
}

geometry_msgs::msg::TwistStamped MpcController::fail(
  const std::string & reason, const std_msgs::msg::Header & header)
{
  const int failures = consecutive_failures_.fetch_add(1) + 1;
  publish_status("stop failure=" + std::to_string(failures) + " reason=" + reason);
  if (failures >= failure_limit_) {
    throw nav2_core::PlannerException(reason);
  }
  return zero_command(header);
}

geometry_msgs::msg::TwistStamped MpcController::retry_stale_path(
  const std::string & reason, const std_msgs::msg::Header & header)
{
  // A changed global path invalidates the old command, but it is an expected
  // retry condition while Nav2 replans.  Publish a validated zero command so
  // the watchdog keeps the robot stopped without consuming the failure limit.
  consecutive_failures_.store(0);
  last_command_time_ = node_->now();
  std::ostringstream status;
  status << "ok generation=" << path_generation_.load()
         << " epoch=" << reset_epoch_.load()
         << " mode=retry wait_reason=path_generation detail=" << reason;
  publish_status(status.str());
  return zero_command(header);
}

void MpcController::publish_status(const std::string & text)
{
  if (status_publisher_ && status_publisher_->is_activated()) {
    std_msgs::msg::String message;
    message.data = text;
    status_publisher_->publish(message);
  }
}

bool MpcController::input_stamp_fresh(const builtin_interfaces::msg::Time & stamp) const
{
  if (!valid_stamp(stamp) || !node_) {
    return false;
  }
  const rclcpp::Time now = node_->now();
  if (now.nanoseconds() <= 0) {
    return false;
  }
  const double age = (now - rclcpp::Time(stamp, RCL_ROS_TIME)).seconds();
  return std::isfinite(age) && age >= -0.05 && age <= ros_age_limit_;
}

void MpcController::on_humans(const hunav_msgs::msg::Agents::SharedPtr message)
{
  if (!input_stamp_fresh(message->header.stamp) || message->header.frame_id.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(input_mutex_);
  if (inputs_.have_humans &&
    stamp_nanoseconds(message->header.stamp) < stamp_nanoseconds(inputs_.humans.header.stamp))
  {
    return;
  }
  inputs_.humans = *message;
  inputs_.human_wall_time = SteadyClock::now();
  inputs_.have_humans = true;
  ++inputs_.human_sequence;
}

void MpcController::on_odom(const nav_msgs::msg::Odometry::SharedPtr message)
{
  if (!input_stamp_fresh(message->header.stamp)) {
    return;
  }
  std::lock_guard<std::mutex> lock(input_mutex_);
  if (inputs_.have_odom &&
    stamp_nanoseconds(message->header.stamp) < stamp_nanoseconds(inputs_.odom.header.stamp))
  {
    return;
  }
  inputs_.odom = *message;
  inputs_.odom_wall_time = SteadyClock::now();
  inputs_.have_odom = true;
  ++inputs_.odom_sequence;
}

void MpcController::on_lidar(const sensor_msgs::msg::LaserScan::SharedPtr message)
{
  if (!input_stamp_fresh(message->header.stamp)) {
    return;
  }
  std::lock_guard<std::mutex> lock(input_mutex_);
  if (inputs_.have_lidar &&
    stamp_nanoseconds(message->header.stamp) < stamp_nanoseconds(inputs_.lidar_stamp))
  {
    return;
  }
  inputs_.lidar_stamp = message->header.stamp;
  inputs_.lidar_wall_time = SteadyClock::now();
  inputs_.have_lidar = true;
  ++inputs_.lidar_sequence;
}

}  // namespace arena_mpc_controller

PLUGINLIB_EXPORT_CLASS(arena_mpc_controller::MpcController, nav2_core::Controller)

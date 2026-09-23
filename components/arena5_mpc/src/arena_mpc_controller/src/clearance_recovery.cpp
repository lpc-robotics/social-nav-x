#include "arena_mpc_controller/clearance_recovery.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <arena_mpc_core/model.hpp>

namespace arena_mpc_controller
{
namespace
{

double clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(value, upper));
}

double effective_linear_limit(
  const arena_mpc_core::Config & config, const arena_mpc_core::Problem & problem)
{
  return problem.linear_speed_limit > 0.0 ? problem.linear_speed_limit : config.max_linear;
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
      clamp(
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

double footprint_clearance(
  const arena_mpc_core::State & robot,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  const arena_mpc_core::Ellipse & obstacle,
  double robot_circumscribed_radius)
{
  const double cosine = std::cos(robot.yaw);
  const double sine = std::sin(robot.yaw);
  const double dx = obstacle.x - robot.x;
  const double dy = obstacle.y - robot.y;
  const double local_x = cosine * dx + sine * dy;
  const double local_y = -sine * dx + cosine * dy;
  const double obstacle_radius =
    std::max(obstacle.semi_major, obstacle.semi_minor) - robot_circumscribed_radius;
  if (!(obstacle_radius > 0.0) || !std::isfinite(obstacle_radius)) {
    return -std::numeric_limits<double>::infinity();
  }
  return point_to_polygon_signed_distance(local_x, local_y, footprint) - obstacle_radius;
}

}  // namespace

ClearanceRecoveryDecision make_clearance_recovery_decision(
  const arena_mpc_core::Config & config,
  const arena_mpc_core::Problem & problem,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  double robot_circumscribed_radius,
  double exit_clearance,
  double desired_linear,
  double maximum_angular,
  double minimum_outward_cosine,
  bool recovery_was_active)
{
  ClearanceRecoveryDecision decision;
  decision.minimum_circle_clearance = std::numeric_limits<double>::infinity();
  const arena_mpc_core::ObstaclePrediction * limiting_obstacle = nullptr;
  bool footprint_safe = footprint.size() >= 3U;
  for (const auto & obstacle : problem.obstacles) {
    if (!obstacle.dynamic || obstacle.samples.empty()) {
      continue;
    }
    const auto & current = obstacle.samples.front();
    const double circle_clearance = arena_mpc_core::ellipse_clearance(
      problem.initial_state, current, config.safe_distance, config.min_axis);
    if (circle_clearance < decision.minimum_circle_clearance) {
      decision.minimum_circle_clearance = circle_clearance;
      decision.limiting_obstacle_id = obstacle.id;
      limiting_obstacle = &obstacle;
    }
    const double exact_clearance = footprint_clearance(
      problem.initial_state, footprint, current, robot_circumscribed_radius);
    footprint_safe = footprint_safe && std::isfinite(exact_clearance) &&
      exact_clearance >= config.safe_distance;
  }
  if (limiting_obstacle == nullptr) {
    decision.minimum_circle_clearance = std::numeric_limits<double>::infinity();
    return decision;
  }
  decision.active = recovery_was_active ?
    decision.minimum_circle_clearance < exit_clearance :
    decision.minimum_circle_clearance < 0.0;
  if (!decision.active) {
    return decision;
  }
  decision.admissible = footprint_safe &&
    decision.minimum_circle_clearance >= -config.max_initial_clearance_violation;
  if (!decision.admissible) {
    return decision;
  }

  const auto & obstacle = limiting_obstacle->samples.front();
  const double escape_yaw = std::atan2(
    problem.initial_state.y - obstacle.y, problem.initial_state.x - obstacle.x);
  double recovery_yaw = escape_yaw;
  if (!problem.reference.empty()) {
    const auto & target = problem.reference.back();
    const double target_x = target.x - problem.initial_state.x;
    const double target_y = target.y - problem.initial_state.y;
    if (std::hypot(target_x, target_y) > 1.0e-9) {
      const double goal_yaw = std::atan2(target_y, target_x);
      const double maximum_deviation = std::acos(clamp(minimum_outward_cosine, 0.0, 1.0));
      const double goal_from_escape = arena_mpc_core::wrap_angle(goal_yaw - escape_yaw);
      recovery_yaw = escape_yaw + clamp(
        goal_from_escape, -maximum_deviation, maximum_deviation);
    }
  }
  decision.heading_error = arena_mpc_core::wrap_angle(recovery_yaw - problem.initial_state.yaw);
  const double interval = problem.first_interval;
  const double linear_limit = effective_linear_limit(config, problem);
  // Turn before translating when the selected escape direction is outside the
  // forward half-plane. This prevents the acceleration ramp from initially
  // driving toward the limiting human while the robot is still reorienting.
  const double heading_scale = std::max(0.0, std::cos(decision.heading_error));
  const double target_linear = std::min(desired_linear, linear_limit) * heading_scale;
  const double target_angular = clamp(
    decision.heading_error / config.dt, -maximum_angular, maximum_angular);
  decision.control.linear = clamp(
    target_linear,
    std::max(config.min_linear,
    problem.measured_control.linear - config.max_linear_accel * interval),
    std::min(linear_limit,
    problem.measured_control.linear + config.max_linear_accel * interval));
  decision.control.angular = clamp(
    target_angular,
    std::max(-config.max_angular,
    problem.measured_control.angular - config.max_angular_accel * interval),
    std::min(config.max_angular,
    problem.measured_control.angular + config.max_angular_accel * interval));
  return decision;
}

}  // namespace arena_mpc_controller

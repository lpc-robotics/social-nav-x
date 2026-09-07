#include "arena_mpc_core/model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace arena_mpc_core
{
namespace
{

double square(double value)
{
  return value * value;
}

double max_abs_state_difference(const State & first, const State & second)
{
  return std::max({
      std::abs(first.x - second.x),
      std::abs(first.y - second.y),
      std::abs(wrap_angle(first.yaw - second.yaw))});
}

bool finite_state(const State & state)
{
  return std::isfinite(state.x) && std::isfinite(state.y) && std::isfinite(state.yaw);
}

bool finite_control(const Control & control)
{
  return std::isfinite(control.linear) && std::isfinite(control.angular);
}

}  // namespace

double wrap_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

State step(const State & state, const Control & control, double dt)
{
  return State{
    state.x + dt * control.linear * std::cos(state.yaw),
    state.y + dt * control.linear * std::sin(state.yaw),
    state.yaw + dt * control.angular};
}

double ellipse_clearance(
  const State & state, const Ellipse & obstacle, double safe_distance, double min_axis)
{
  if (!finite_state(state) || !std::isfinite(obstacle.x) || !std::isfinite(obstacle.y) ||
    !std::isfinite(obstacle.semi_major) || !std::isfinite(obstacle.semi_minor) ||
    !std::isfinite(obstacle.yaw) || !std::isfinite(safe_distance) ||
    !std::isfinite(min_axis) || min_axis <= 0.0)
  {
    throw std::invalid_argument("ellipse_clearance received non-finite or invalid input");
  }
  if (obstacle.semi_major < min_axis || obstacle.semi_minor < min_axis) {
    throw std::invalid_argument("ellipse axes are below min_axis");
  }

  const double dx = state.x - obstacle.x;
  const double dy = state.y - obstacle.y;
  const double cosine = std::cos(obstacle.yaw);
  const double sine = std::sin(obstacle.yaw);
  const double local_x = cosine * dx + sine * dy;
  const double local_y = -sine * dx + cosine * dy;
  constexpr double squared_norm_regularizer = 1.0e-12;
  const double normalized = std::sqrt(
    square(local_x / obstacle.semi_major) + square(local_y / obstacle.semi_minor) +
    squared_norm_regularizer) - std::sqrt(squared_norm_regularizer);
  return obstacle.semi_minor * (normalized - 1.0) - safe_distance;
}

void validate_problem(const Config & config, const Problem & problem)
{
  if (config.horizon == 0U || !std::isfinite(config.dt) || config.dt <= 0.0 ||
    !std::isfinite(config.gamma) || config.gamma < 0.0 || config.gamma > 1.0 ||
    !std::isfinite(config.safe_distance) || config.safe_distance < 0.0 ||
    !std::isfinite(config.min_axis) || config.min_axis <= 0.0 ||
    config.max_linear < config.min_linear || config.max_angular <= 0.0 ||
    config.max_linear_accel <= 0.0 || config.max_angular_accel <= 0.0)
  {
    throw std::invalid_argument("invalid MPC configuration");
  }
  if (!finite_state(problem.initial_state) || !finite_control(problem.measured_control) ||
    !std::isfinite(problem.first_interval) || problem.first_interval <= 0.0 ||
    problem.first_interval > config.dt)
  {
    throw std::invalid_argument("invalid initial state, odometry control, or first interval");
  }
  if (problem.reference.size() != config.horizon + 1U) {
    throw std::invalid_argument("reference must contain horizon + 1 states");
  }
  if (config.max_nlp_dynamic_obstacles > config.max_dynamic_obstacles ||
    config.max_nlp_static_obstacles > config.max_static_obstacles)
  {
    throw std::invalid_argument("NLP obstacle capacity exceeds input capacity");
  }
  for (const auto & state : problem.reference) {
    if (!finite_state(state)) {
      throw std::invalid_argument("reference contains a non-finite state");
    }
  }

  std::size_t dynamic_count = 0U;
  std::size_t static_count = 0U;
  for (const auto & obstacle : problem.obstacles) {
    if (obstacle.dynamic) {
      ++dynamic_count;
    } else {
      ++static_count;
    }
    if (obstacle.samples.size() != config.horizon + 1U) {
      throw std::invalid_argument("each obstacle must contain horizon + 1 samples");
    }
    for (const auto & sample : obstacle.samples) {
      if (!std::isfinite(sample.x) || !std::isfinite(sample.y) ||
        !std::isfinite(sample.semi_major) || !std::isfinite(sample.semi_minor) ||
        !std::isfinite(sample.yaw) || sample.semi_major < config.min_axis ||
        sample.semi_minor < config.min_axis)
      {
        throw std::invalid_argument("obstacle contains non-finite or degenerate geometry");
      }
    }
  }
  if (dynamic_count > config.max_dynamic_obstacles || static_count > config.max_static_obstacles) {
    throw std::length_error("obstacle capacity exceeded");
  }
}

Evaluation evaluate(
  const Config & config, const Problem & problem, const Trajectory & trajectory)
{
  validate_problem(config, problem);
  if (trajectory.states.size() != config.horizon + 1U ||
    trajectory.controls.size() != config.horizon ||
    trajectory.barrier_slack.size() != problem.obstacles.size() * config.horizon)
  {
    throw std::invalid_argument("trajectory dimensions do not match problem");
  }

  Evaluation result;
  result.max_initial_residual = max_abs_state_difference(
    trajectory.states.front(), problem.initial_state);

  for (std::size_t k = 0; k < config.horizon; ++k) {
    const auto & state = trajectory.states[k];
    const auto & control = trajectory.controls[k];
    const auto & reference = problem.reference[k];
    if (!finite_state(state) || !finite_control(control)) {
      throw std::invalid_argument("trajectory contains a non-finite value");
    }

    const double position_weight = 1.0 + 0.05 * static_cast<double>(k);
    const double yaw_weight = 0.02 + 0.005 * static_cast<double>(k);
    result.objective += 0.1 * (
      position_weight * square(state.x - reference.x) +
      position_weight * square(state.y - reference.y) +
      yaw_weight * square(wrap_angle(state.yaw - reference.yaw)));
    result.objective += 0.1 * square(control.linear) + 0.02 * square(control.angular);

    const State expected = step(state, control, config.dt);
    result.max_dynamics_residual = std::max(
      result.max_dynamics_residual,
      max_abs_state_difference(trajectory.states[k + 1U], expected));

    result.max_bound_violation = std::max({
        result.max_bound_violation,
        config.min_linear - control.linear,
        control.linear - config.max_linear,
        std::abs(control.angular) - config.max_angular});

    const Control previous = k == 0U ? problem.measured_control : trajectory.controls[k - 1U];
    const double interval = k == 0U ? problem.first_interval : config.dt;
    result.max_acceleration_violation = std::max({
        result.max_acceleration_violation,
        std::abs(control.linear - previous.linear) - config.max_linear_accel * interval,
        std::abs(control.angular - previous.angular) - config.max_angular_accel * interval});
  }

  const auto & terminal = trajectory.states.back();
  const auto & terminal_reference = problem.reference.back();
  result.objective += config.terminal_weight * (
    square(terminal.x - terminal_reference.x) +
    square(terminal.y - terminal_reference.y) +
    0.02 * square(wrap_angle(terminal.yaw - terminal_reference.yaw)));

  for (std::size_t obstacle_index = 0; obstacle_index < problem.obstacles.size(); ++obstacle_index) {
    const auto & obstacle = problem.obstacles[obstacle_index];
    double previous_clearance = ellipse_clearance(
      trajectory.states.front(), obstacle.samples.front(), config.safe_distance, config.min_axis);
    result.max_geometry_violation = std::max(result.max_geometry_violation, -previous_clearance);
    for (std::size_t k = 0; k < config.horizon; ++k) {
      const double next_clearance = ellipse_clearance(
        trajectory.states[k + 1U], obstacle.samples[k + 1U], config.safe_distance,
        config.min_axis);
      const double slack = trajectory.barrier_slack[obstacle_index * config.horizon + k];
      if (!std::isfinite(slack)) {
        throw std::invalid_argument("trajectory contains non-finite slack");
      }
      result.max_geometry_violation = std::max(result.max_geometry_violation, -next_clearance);
      result.max_barrier_violation = std::max(
        result.max_barrier_violation,
        config.gamma * previous_clearance - next_clearance - slack);
      result.objective += config.slack_weight * square(slack);
      previous_clearance = next_clearance;
    }
  }

  result.max_bound_violation = std::max(0.0, result.max_bound_violation);
  result.max_acceleration_violation = std::max(0.0, result.max_acceleration_violation);
  result.max_geometry_violation = std::max(0.0, result.max_geometry_violation);
  result.max_barrier_violation = std::max(0.0, result.max_barrier_violation);
  return result;
}

}  // namespace arena_mpc_core

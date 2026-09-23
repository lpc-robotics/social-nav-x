#include "arena_mpc_controller/reference_builder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace arena_mpc_controller
{
namespace
{

double distance(
  const arena_mpc_core::State & first,
  const arena_mpc_core::State & second)
{
  return std::hypot(first.x - second.x, first.y - second.y);
}

double terminal_path_yaw(
  const std::vector<arena_mpc_core::State> & path,
  const arena_mpc_core::State & robot)
{
  for (std::size_t index = path.size(); index > 1U; --index) {
    const auto & first = path[index - 2U];
    const auto & second = path[index - 1U];
    const double dx = second.x - first.x;
    const double dy = second.y - first.y;
    if (std::hypot(dx, dy) > 1.0e-9) {
      return std::atan2(dy, dx);
    }
  }
  const double dx = path.back().x - robot.x;
  const double dy = path.back().y - robot.y;
  return std::hypot(dx, dy) > 1.0e-9 ? std::atan2(dy, dx) : path.back().yaw;
}

}  // namespace

ReferenceBuildResult build_path_reference(
  const std::vector<arena_mpc_core::State> & path,
  const arena_mpc_core::State & robot,
  std::size_t horizon,
  double spacing,
  double xy_goal_tolerance,
  bool position_already_latched)
{
  if (path.empty()) {
    throw std::invalid_argument("reference path must not be empty");
  }
  if (!std::isfinite(spacing) || spacing <= 0.0) {
    throw std::invalid_argument("reference spacing must be positive and finite");
  }
  if (!std::isfinite(xy_goal_tolerance) || xy_goal_tolerance <= 0.0) {
    throw std::invalid_argument("XY goal tolerance must be positive and finite");
  }

  ReferenceBuildResult result;
  result.goal_distance = distance(robot, path.back());
  result.position_latched =
    position_already_latched || result.goal_distance <= xy_goal_tolerance;
  result.states.reserve(horizon + 1U);
  if (result.position_latched) {
    result.states.assign(horizon + 1U, path.back());
    return result;
  }

  std::size_t nearest = 0U;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < path.size(); ++index) {
    const double candidate = distance(robot, path[index]);
    if (candidate < nearest_distance) {
      nearest_distance = candidate;
      nearest = index;
    }
  }

  std::vector<double> cumulative(path.size() - nearest, 0.0);
  for (std::size_t index = 1U; index < cumulative.size(); ++index) {
    cumulative[index] = cumulative[index - 1U] +
      distance(path[nearest + index - 1U], path[nearest + index]);
  }
  const double arrival_yaw = terminal_path_yaw(path, robot);

  for (std::size_t step_index = 0U; step_index <= horizon; ++step_index) {
    const double wanted = spacing * static_cast<double>(step_index);
    const auto upper = std::lower_bound(cumulative.begin(), cumulative.end(), wanted);
    const std::size_t local_index = upper == cumulative.end() ? cumulative.size() - 1U :
      static_cast<std::size_t>(std::distance(cumulative.begin(), upper));
    arena_mpc_core::State reference;
    if (local_index == 0U || cumulative[local_index] <= wanted) {
      reference = path[nearest + local_index];
    } else {
      const double lower_distance = cumulative[local_index - 1U];
      const double segment = cumulative[local_index] - lower_distance;
      const double ratio = segment > 1.0e-9 ? (wanted - lower_distance) / segment : 0.0;
      const auto & first = path[nearest + local_index - 1U];
      const auto & second = path[nearest + local_index];
      reference.x = first.x + ratio * (second.x - first.x);
      reference.y = first.y + ratio * (second.y - first.y);
    }
    const std::size_t tangent_index = std::min(local_index + 1U, cumulative.size() - 1U);
    const auto & tangent_target = path[nearest + tangent_index];
    const double dx = tangent_target.x - reference.x;
    const double dy = tangent_target.y - reference.y;
    reference.yaw = std::hypot(dx, dy) > 1.0e-9 ? std::atan2(dy, dx) : arrival_yaw;
    result.states.push_back(reference);
  }
  return result;
}

}  // namespace arena_mpc_controller

#include "arena_mpc_controller/clearance_recovery.hpp"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace
{

bool require(bool condition, const std::string & message)
{
  if (!condition) {
    std::cerr << "CLEARANCE_RECOVERY_TEST_FAILED " << message << '\n';
  }
  return condition;
}

std::vector<geometry_msgs::msg::Point> footprint()
{
  std::vector<geometry_msgs::msg::Point> result(4U);
  result[0].x = 0.24; result[0].y = 0.22;
  result[1].x = 0.24; result[1].y = -0.22;
  result[2].x = -0.24; result[2].y = -0.22;
  result[3].x = -0.24; result[3].y = 0.22;
  return result;
}

arena_mpc_core::ObstaclePrediction obstacle(double x, double y, double radius)
{
  arena_mpc_core::ObstaclePrediction result;
  result.id = 3U;
  result.dynamic = true;
  result.samples.assign(26U, {x, y, radius, radius, 0.0});
  return result;
}

}  // namespace

int main()
{
  arena_mpc_core::Config config;
  config.safe_distance = 0.35;
  config.max_initial_clearance_violation = 0.05;
  const double robot_radius = std::hypot(0.24, 0.22);
  const double obstacle_radius = 0.4 + robot_radius + 0.05;
  arena_mpc_core::Problem problem;
  problem.initial_state = {5.731477737426758, 6.052096366882324, 2.349259933692989};
  problem.measured_control = {0.0, 0.0};
  problem.first_interval = 0.1;
  problem.linear_speed_limit = 0.8;
  problem.obstacles.push_back(
    obstacle(6.526482323325462, 6.822655123135009, obstacle_radius));

  const auto start = arena_mpc_controller::make_clearance_recovery_decision(
    config, problem, footprint(), robot_radius, 0.10, 0.40, 1.0, false);
  if (!require(start.active && start.admissible, "live fault geometry was not admitted") ||
    !require(start.minimum_circle_clearance < 0.0, "live fault was not reproduced") ||
    !require(std::abs(start.control.linear - 0.2) < 1.0e-12, "linear acceleration bound lost") ||
    !require(std::abs(start.control.angular - 0.32) < 1.0e-12, "angular acceleration bound lost"))
  {
    return 1;
  }

  problem.initial_state.x = 5.60;
  problem.initial_state.y = 5.92;
  const auto completed = arena_mpc_controller::make_clearance_recovery_decision(
    config, problem, footprint(), robot_radius, 0.10, 0.40, 1.0, true);
  if (!require(!completed.active, "recovery did not release after restoring clearance")) {
    return 2;
  }

  problem.initial_state = {5.9, 6.3, 0.0};
  const auto unsafe = arena_mpc_controller::make_clearance_recovery_decision(
    config, problem, footprint(), robot_radius, 0.10, 0.40, 1.0, false);
  if (!require(unsafe.active && !unsafe.admissible, "unsafe initial footprint was admitted")) {
    return 3;
  }

  std::cout << "CLEARANCE_RECOVERY_TEST_OK circle=" << start.minimum_circle_clearance
            << " v=" << start.control.linear << " w=" << start.control.angular << '\n';
  return 0;
}

#include <arena_mpc_core/model.hpp>
#include <arena_mpc_core/solver.hpp>

#include <cmath>
#include <iostream>
#include <string>

namespace
{

arena_mpc_core::Problem straight_problem(const arena_mpc_core::Config & config)
{
  arena_mpc_core::Problem problem;
  problem.initial_state = {0.0, 0.0, M_PI - 0.01};
  problem.measured_control = {0.0, 0.0};
  problem.first_interval = config.dt;
  problem.reference.reserve(config.horizon + 1U);
  for (std::size_t k = 0; k <= config.horizon; ++k) {
    problem.reference.push_back({-0.02 * static_cast<double>(k), 0.0, -M_PI + 0.01});
  }
  return problem;
}

arena_mpc_core::ObstaclePrediction repeated_obstacle(
  const arena_mpc_core::Config & config, double x, double y, double major, double minor,
  double yaw)
{
  arena_mpc_core::ObstaclePrediction obstacle;
  obstacle.id = 1U;
  obstacle.dynamic = true;
  obstacle.samples.assign(config.horizon + 1U, {x, y, major, minor, yaw});
  return obstacle;
}

bool require(bool condition, const std::string & message)
{
  if (!condition) {
    std::cerr << "SOLVER_CASE_FAILED " << message << '\n';
  }
  return condition;
}

}  // namespace

int main()
{
  using arena_mpc_core::SolveCode;

  arena_mpc_core::Config config;
  config.horizon = 25U;
  config.solver_budget_seconds = 0.060;
  config.max_iterations = 100;
  arena_mpc_core::Solver solver(config);

  auto base = straight_problem(config);
  const auto cold = solver.solve(base, false);
  if (!require(cold.code == SolveCode::Success, "cold solve: " + cold.status) ||
    !require(cold.command_valid, "cold command invalid") ||
    !require(std::abs(cold.trajectory.controls.front().linear) <= 0.2001,
      "first linear acceleration is not anchored to odometry") ||
    !require(cold.evaluation.max_dynamics_residual <= 1.0e-3, "dynamics residual") ||
    !require(cold.evaluation.max_bound_violation <= 1.0e-3, "control bounds"))
  {
    return 1;
  }

  const auto warm = solver.solve(base, true);
  if (!require(warm.code == SolveCode::Success, "warm solve: " + warm.status) ||
    !require(warm.evaluation.max_dynamics_residual <= 1.0e-3, "warm dynamics residual"))
  {
    return 2;
  }

  auto horizontal = base;
  horizontal.obstacles.push_back(repeated_obstacle(config, 2.0, 1.0, 0.5, 0.2, 0.0));
  const auto horizontal_result = solver.solve(horizontal, false);
  if (!require(horizontal_result.code == SolveCode::Success,
      "horizontal ellipse: " + horizontal_result.status))
  {
    return 3;
  }

  auto vertical = base;
  vertical.obstacles.push_back(repeated_obstacle(config, 2.0, 1.0, 0.5, 0.2, M_PI_2));
  const auto vertical_result = solver.solve(vertical, false);
  if (!require(vertical_result.code == SolveCode::Success,
      "vertical ellipse: " + vertical_result.status))
  {
    return 4;
  }

  auto degenerate = base;
  degenerate.obstacles.push_back(repeated_obstacle(config, 2.0, 1.0, 0.0, 0.2, 0.0));
  const auto degenerate_result = solver.solve(degenerate, false);
  if (!require(degenerate_result.code == SolveCode::InvalidInput,
      "degenerate ellipse was not rejected"))
  {
    return 5;
  }

  auto collision = base;
  collision.obstacles.push_back(repeated_obstacle(config, 0.0, 0.0, 0.3, 0.3, 0.0));
  const auto collision_result = solver.solve(collision, false);
  if (!require(collision_result.code != SolveCode::Success && !collision_result.command_valid,
      "initial overlap produced a valid command"))
  {
    return 6;
  }

  auto overflow = base;
  for (std::size_t i = 0; i <= config.max_dynamic_obstacles; ++i) {
    auto obstacle = repeated_obstacle(config, 10.0 + static_cast<double>(i), 10.0, 0.3, 0.3, 0.0);
    obstacle.id = i;
    overflow.obstacles.push_back(obstacle);
  }
  const auto overflow_result = solver.solve(overflow, false);
  if (!require(overflow_result.code == SolveCode::CapacityExceeded,
      "dynamic capacity overflow was not rejected"))
  {
    return 7;
  }

  auto relevant_overflow = base;
  for (std::size_t i = 0; i <= config.max_nlp_dynamic_obstacles; ++i) {
    auto obstacle = repeated_obstacle(
      config, -0.3, (i % 2U == 0U ? 0.65 : -0.65), 0.2, 0.2, 0.0);
    obstacle.id = 100U + i;
    relevant_overflow.obstacles.push_back(obstacle);
  }
  const auto relevant_overflow_result = solver.solve(relevant_overflow, false);
  if (!require(relevant_overflow_result.code == SolveCode::CapacityExceeded,
      "relevant NLP capacity overflow was not rejected"))
  {
    return 8;
  }

  std::cout << "SOLVER_CASES_OK cold_ms=" << cold.timing.solve_ms
            << " warm_ms=" << warm.timing.solve_ms
            << " first_linear=" << warm.trajectory.controls.front().linear << '\n';
  return 0;
}

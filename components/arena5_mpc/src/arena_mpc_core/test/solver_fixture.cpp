#include <arena_mpc_core/solver.hpp>

#include <iomanip>
#include <iostream>

int main()
{
  arena_mpc_core::Config config;
  arena_mpc_core::Problem problem;
  problem.initial_state = {0.0, 0.0, 0.0};
  problem.measured_control = {0.0, 0.0};
  problem.first_interval = config.dt;
  for (std::size_t k = 0; k <= config.horizon; ++k) {
    problem.reference.push_back({0.02 * static_cast<double>(k), 0.0, 0.0});
  }
  arena_mpc_core::Solver solver(config);
  const auto result = solver.solve(problem, false);
  if (!result.command_valid) {
    std::cerr << "SOLVER_FIXTURE_FAILED status=" << result.status << '\n';
    return 1;
  }
  std::cout << std::setprecision(17)
            << "linear=" << result.trajectory.controls.front().linear << '\n'
            << "angular=" << result.trajectory.controls.front().angular << '\n'
            << "objective=" << result.evaluation.objective << '\n';
  return 0;
}

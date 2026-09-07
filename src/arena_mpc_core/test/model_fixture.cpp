#include <arena_mpc_core/model.hpp>

#include <cmath>
#include <iomanip>
#include <iostream>

int main()
{
  using arena_mpc_core::Config;
  using arena_mpc_core::Control;
  using arena_mpc_core::Ellipse;
  using arena_mpc_core::ObstaclePrediction;
  using arena_mpc_core::Problem;
  using arena_mpc_core::State;
  using arena_mpc_core::Trajectory;

  Config config;
  config.horizon = 3U;
  config.dt = 0.1;
  config.gamma = 0.2;
  config.safe_distance = 0.3;
  config.terminal_weight = 1.7;
  config.slack_weight = 50.0;

  Problem problem;
  problem.initial_state = State{0.2, -0.1, M_PI - 0.01};
  problem.measured_control = Control{0.05, -0.1};
  problem.first_interval = 0.08;
  problem.reference = {
    State{0.2, -0.1, -M_PI + 0.01},
    State{0.18, -0.1, -M_PI + 0.02},
    State{0.16, -0.101, -M_PI + 0.03},
    State{0.14, -0.102, -M_PI + 0.04}};
  ObstaclePrediction obstacle;
  obstacle.id = 7U;
  obstacle.dynamic = true;
  obstacle.samples = {
    Ellipse{1.0, 0.5, 0.45, 0.25, 0.4},
    Ellipse{0.99, 0.5, 0.45, 0.25, 0.4},
    Ellipse{0.98, 0.5, 0.45, 0.25, 0.4},
    Ellipse{0.97, 0.5, 0.45, 0.25, 0.4}};
  problem.obstacles.push_back(obstacle);

  Trajectory trajectory;
  trajectory.states.push_back(problem.initial_state);
  trajectory.controls = {Control{0.10, 0.05}, Control{0.12, 0.08}, Control{0.11, -0.02}};
  for (const auto & control : trajectory.controls) {
    trajectory.states.push_back(arena_mpc_core::step(trajectory.states.back(), control, config.dt));
  }
  trajectory.states[2].x += 2.5e-5;
  trajectory.barrier_slack = {0.01, 0.02, 0.03};

  const auto evaluation = arena_mpc_core::evaluate(config, problem, trajectory);
  const double clearance = arena_mpc_core::ellipse_clearance(
    trajectory.states[1], obstacle.samples[1], config.safe_distance, config.min_axis);

  std::cout << std::setprecision(17)
            << "wrap=" << arena_mpc_core::wrap_angle(2.0 * M_PI - 0.03) << '\n'
            << "clearance=" << clearance << '\n'
            << "objective=" << evaluation.objective << '\n'
            << "initial=" << evaluation.max_initial_residual << '\n'
            << "dynamics=" << evaluation.max_dynamics_residual << '\n'
            << "geometry=" << evaluation.max_geometry_violation << '\n'
            << "barrier=" << evaluation.max_barrier_violation << '\n'
            << "acceleration=" << evaluation.max_acceleration_violation << '\n'
            << "bounds=" << evaluation.max_bound_violation << '\n';
  return 0;
}

#include <arena_mpc_core/solver.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace
{

struct Summary
{
  double p50{0.0};
  double p95{0.0};
  double p99{0.0};
  double maximum{0.0};
  std::size_t success{0U};
  std::size_t timeout{0U};
};

double percentile(const std::vector<double> & sorted, double probability)
{
  if (sorted.empty()) {
    return 0.0;
  }
  const double position = probability * static_cast<double>(sorted.size() - 1U);
  const auto lower = static_cast<std::size_t>(std::floor(position));
  const auto upper = static_cast<std::size_t>(std::ceil(position));
  const double fraction = position - static_cast<double>(lower);
  return sorted[lower] * (1.0 - fraction) + sorted[upper] * fraction;
}

arena_mpc_core::Problem make_problem(
  const arena_mpc_core::Config & config, std::size_t dynamic_count,
  std::size_t static_count, const std::string & condition)
{
  arena_mpc_core::Problem problem;
  problem.initial_state = {0.0, 0.0, 0.0};
  problem.measured_control = {0.0, 0.0};
  problem.first_interval = config.dt;
  for (std::size_t k = 0; k <= config.horizon; ++k) {
    problem.reference.push_back({0.02 * static_cast<double>(k), 0.0, 0.0});
  }
  const std::size_t total = dynamic_count + static_count;
  for (std::size_t obstacle_index = 0; obstacle_index < total; ++obstacle_index) {
    arena_mpc_core::ObstaclePrediction obstacle;
    obstacle.id = obstacle_index + 1U;
    obstacle.dynamic = obstacle_index < dynamic_count;
    const std::size_t type_index = obstacle.dynamic ? obstacle_index : obstacle_index - dynamic_count;
    const bool nlp_relevant = obstacle.dynamic ?
      type_index < config.max_nlp_dynamic_obstacles :
      type_index < config.max_nlp_static_obstacles;
    double x = nlp_relevant ? 0.55 + 0.02 * static_cast<double>(type_index % 8U) :
      2.0 + 0.15 * static_cast<double>(obstacle_index % 16U);
    double y = nlp_relevant ? (type_index % 2U == 0U ? 0.85 : -0.85) :
      1.5 + 0.15 * static_cast<double>(obstacle_index / 16U);
    if (obstacle_index == 0U && condition == "critical") {
      x = 0.55;
      y = 0.53;
    } else if (obstacle_index == 0U && condition == "infeasible") {
      x = 0.0;
      y = 0.0;
    }
    for (std::size_t k = 0; k <= config.horizon; ++k) {
      const double drift = obstacle.dynamic ? -0.002 * static_cast<double>(k) : 0.0;
      obstacle.samples.push_back({x + drift, y, 0.28, 0.22, 0.1});
    }
    problem.obstacles.push_back(std::move(obstacle));
  }
  return problem;
}

Summary run_case(
  arena_mpc_core::Solver & solver, const arena_mpc_core::Problem & problem,
  std::size_t iterations)
{
  Summary summary;
  std::vector<double> samples;
  samples.reserve(iterations);
  for (std::size_t iteration = 0; iteration < iterations; ++iteration) {
    const auto result = solver.solve(problem, iteration != 0U);
    samples.push_back(result.timing.solve_ms);
    if (result.code == arena_mpc_core::SolveCode::Success) {
      ++summary.success;
    }
    if (result.code == arena_mpc_core::SolveCode::Timeout) {
      ++summary.timeout;
    }
  }
  std::sort(samples.begin(), samples.end());
  summary.p50 = percentile(samples, 0.50);
  summary.p95 = percentile(samples, 0.95);
  summary.p99 = percentile(samples, 0.99);
  summary.maximum = samples.empty() ? 0.0 : samples.back();
  return summary;
}

void print_case(
  const std::string & layout, const std::string & load, const std::string & condition,
  std::size_t iterations, const arena_mpc_core::Result & cold, const Summary & summary)
{
  std::cout << layout << ',' << load << ',' << condition << ',' << iterations << ','
            << std::fixed << std::setprecision(6)
            << cold.timing.graph_build_ms << ',' << cold.timing.parameter_update_ms << ','
            << cold.timing.solve_ms << ',' << cold.timing.postcheck_ms << ','
            << summary.p50 << ',' << summary.p95 << ',' << summary.p99 << ',' << summary.maximum
            << ',' << summary.success << ',' << summary.timeout << ','
            << cold.evaluation.max_dynamics_residual << ','
            << cold.evaluation.max_geometry_violation << ','
            << cold.evaluation.max_barrier_violation << ','
            << static_cast<int>(cold.code) << ',' << cold.status << '\n';
}

}  // namespace

int main(int argc, char ** argv)
{
  std::size_t iterations = 1000U;
  if (argc == 2) {
    iterations = static_cast<std::size_t>(std::stoul(argv[1]));
  }
  std::cout << "layout,load,condition,iterations,graph_build_ms,cold_parameter_ms,cold_solve_ms,"
               "cold_postcheck_ms,warm_p50_ms,warm_p95_ms,warm_p99_ms,warm_max_ms,success,timeout,"
               "dynamics_residual,geometry_violation,barrier_violation,cold_code,cold_status\n";

  for (const auto layout : {arena_mpc_core::SolverLayout::ExactActive,
      arena_mpc_core::SolverLayout::FixedMasked})
  {
    const std::string layout_name = layout == arena_mpc_core::SolverLayout::ExactActive ?
      "exact_active" : "fixed_masked";
    for (const auto & item : std::vector<std::pair<std::string, std::pair<std::size_t, std::size_t>>>{
        {"none", {0U, 0U}}, {"six_dynamic", {6U, 0U}}, {"maximum", {32U, 128U}}})
    {
      for (const std::string condition : {"feasible", "critical", "infeasible"}) {
        if (item.first == "none" && condition != "feasible") {
          continue;
        }
        arena_mpc_core::Config config;
        config.layout = layout;
        arena_mpc_core::Solver solver(config);
        const auto problem = make_problem(config, item.second.first, item.second.second, condition);
        const auto cold = solver.solve(problem, false);
        const Summary summary = run_case(solver, problem, iterations);
        print_case(layout_name, item.first, condition, iterations, cold, summary);
      }
    }
  }
  return 0;
}

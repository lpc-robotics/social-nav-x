#include "arena_mpc_core/solver.hpp"

#include <casadi/casadi.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "arena_mpc_core/model.hpp"

namespace arena_mpc_core
{
namespace
{

using Clock = std::chrono::steady_clock;

double milliseconds(Clock::time_point start, Clock::time_point end)
{
  return std::chrono::duration<double, std::milli>(end - start).count();
}

std::size_t state_index(std::size_t step_index, std::size_t component)
{
  return 3U * step_index + component;
}

std::size_t control_base(const Config & config)
{
  return 3U * (config.horizon + 1U);
}

std::size_t control_index(
  const Config & config, std::size_t step_index, std::size_t component)
{
  return control_base(config) + 2U * step_index + component;
}

std::size_t parameter_reference_base()
{
  return 6U;
}

std::size_t parameter_obstacle_base(const Config & config)
{
  return parameter_reference_base() + 3U * (config.horizon + 1U);
}

std::size_t obstacle_parameter_stride(const Config & config)
{
  return 1U + 5U * (config.horizon + 1U);
}

std::size_t obstacle_parameter_index(
  const Config & config, std::size_t obstacle_index, std::size_t sample_index,
  std::size_t component)
{
  return parameter_obstacle_base(config) +
         obstacle_index * obstacle_parameter_stride(config) + 1U +
         5U * sample_index + component;
}

double clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(value, upper));
}

double effective_linear_limit(const Config & config, const Problem & problem)
{
  return problem.linear_speed_limit > 0.0 ? problem.linear_speed_limit : config.max_linear;
}

std::string stat_string(const casadi::Dict & stats, const std::string & key)
{
  const auto item = stats.find(key);
  return item == stats.end() ? std::string{} : item->second.to_string();
}

int stat_int(const casadi::Dict & stats, const std::string & key)
{
  const auto item = stats.find(key);
  return item == stats.end() ? 0 : static_cast<int>(item->second.to_int());
}

bool stat_bool(const casadi::Dict & stats, const std::string & key)
{
  const auto item = stats.find(key);
  return item != stats.end() && item->second.to_bool();
}

struct Graph
{
  casadi::Function solver;
  std::size_t slots{0U};
  std::size_t variable_count{0U};
  std::size_t parameter_count{0U};
  std::size_t first_linear_acceleration_constraint{0U};
  std::size_t first_angular_acceleration_constraint{0U};
  std::vector<double> lower_variable;
  std::vector<double> upper_variable;
  std::vector<double> lower_constraint;
  std::vector<double> upper_constraint;
  std::vector<double> last_solution;
  double build_ms{0.0};
};

casadi::MX symbolic_clearance(
  const Config & config, const casadi::MX & state_x, const casadi::MX & state_y,
  const casadi::MX & parameters, std::size_t obstacle_index, std::size_t sample_index)
{
  const casadi::MX dx = state_x -
    parameters(obstacle_parameter_index(config, obstacle_index, sample_index, 0U));
  const casadi::MX dy = state_y -
    parameters(obstacle_parameter_index(config, obstacle_index, sample_index, 1U));
  const casadi::MX major =
    parameters(obstacle_parameter_index(config, obstacle_index, sample_index, 2U));
  const casadi::MX minor =
    parameters(obstacle_parameter_index(config, obstacle_index, sample_index, 3U));
  const casadi::MX yaw =
    parameters(obstacle_parameter_index(config, obstacle_index, sample_index, 4U));
  const casadi::MX local_x = casadi::MX::cos(yaw) * dx + casadi::MX::sin(yaw) * dy;
  const casadi::MX local_y = -casadi::MX::sin(yaw) * dx + casadi::MX::cos(yaw) * dy;
  constexpr double squared_norm_regularizer = 1.0e-12;
  const casadi::MX normalized = casadi::MX::sqrt(
    local_x * local_x / (major * major) + local_y * local_y / (minor * minor) +
    squared_norm_regularizer) - std::sqrt(squared_norm_regularizer);
  return minor * (normalized - 1.0) - config.safe_distance;
}

std::unique_ptr<Graph> build_graph(const Config & config, std::size_t slots)
{
  static std::atomic<std::uint64_t> graph_sequence{0U};
  const auto build_start = Clock::now();
  auto graph = std::make_unique<Graph>();
  graph->slots = slots;
  graph->variable_count = control_base(config) + 2U * config.horizon;
  graph->parameter_count = parameter_obstacle_base(config) +
    slots * obstacle_parameter_stride(config);
  graph->lower_variable.assign(graph->variable_count, -casadi::inf);
  graph->upper_variable.assign(graph->variable_count, casadi::inf);

  const casadi::MX variables = casadi::MX::sym("z", graph->variable_count);
  const casadi::MX parameters = casadi::MX::sym("p", graph->parameter_count);
  casadi::MX objective = 0.0;
  std::vector<casadi::MX> constraints;
  std::vector<double> lower_constraint;
  std::vector<double> upper_constraint;

  const auto append_constraint = [&](const casadi::MX & expression, double lower, double upper) {
      constraints.push_back(expression);
      lower_constraint.push_back(lower);
      upper_constraint.push_back(upper);
    };

  for (std::size_t component = 0; component < 3U; ++component) {
    append_constraint(variables(state_index(0U, component)) - parameters(component), 0.0, 0.0);
  }

  for (std::size_t k = 0; k < config.horizon; ++k) {
    const casadi::MX x = variables(state_index(k, 0U));
    const casadi::MX y = variables(state_index(k, 1U));
    const casadi::MX yaw = variables(state_index(k, 2U));
    const casadi::MX linear = variables(control_index(config, k, 0U));
    const casadi::MX angular = variables(control_index(config, k, 1U));
    const std::size_t reference_index = parameter_reference_base() + 3U * k;
    const casadi::MX dx = x - parameters(reference_index);
    const casadi::MX dy = y - parameters(reference_index + 1U);
    const casadi::MX yaw_delta = yaw - parameters(reference_index + 2U);
    const casadi::MX wrapped_yaw = casadi::MX::atan2(
      casadi::MX::sin(yaw_delta), casadi::MX::cos(yaw_delta));
    const double position_weight = 1.0 + 0.05 * static_cast<double>(k);
    const double yaw_weight = 0.02 + 0.005 * static_cast<double>(k);
    objective += 0.1 * (
      position_weight * dx * dx + position_weight * dy * dy +
      yaw_weight * wrapped_yaw * wrapped_yaw);
    objective += 0.1 * linear * linear + 0.02 * angular * angular;

    graph->lower_variable[control_index(config, k, 0U)] = config.min_linear;
    graph->upper_variable[control_index(config, k, 0U)] = config.max_linear;
    graph->lower_variable[control_index(config, k, 1U)] = -config.max_angular;
    graph->upper_variable[control_index(config, k, 1U)] = config.max_angular;

    const casadi::MX previous_linear = k == 0U ? parameters(3U) :
      variables(control_index(config, k - 1U, 0U));
    const casadi::MX previous_angular = k == 0U ? parameters(4U) :
      variables(control_index(config, k - 1U, 1U));
    if (k == 0U) {
      graph->first_linear_acceleration_constraint = constraints.size();
    }
    append_constraint(
      linear - previous_linear,
      -config.max_linear_accel * config.dt, config.max_linear_accel * config.dt);
    if (k == 0U) {
      graph->first_angular_acceleration_constraint = constraints.size();
    }
    append_constraint(
      angular - previous_angular,
      -config.max_angular_accel * config.dt, config.max_angular_accel * config.dt);

    append_constraint(
      variables(state_index(k + 1U, 0U)) - x - config.dt * linear * casadi::MX::cos(yaw),
      0.0, 0.0);
    append_constraint(
      variables(state_index(k + 1U, 1U)) - y - config.dt * linear * casadi::MX::sin(yaw),
      0.0, 0.0);
    append_constraint(
      variables(state_index(k + 1U, 2U)) - yaw - config.dt * angular,
      0.0, 0.0);
  }

  const std::size_t terminal_reference = parameter_reference_base() + 3U * config.horizon;
  const casadi::MX terminal_dx = variables(state_index(config.horizon, 0U)) -
    parameters(terminal_reference);
  const casadi::MX terminal_dy = variables(state_index(config.horizon, 1U)) -
    parameters(terminal_reference + 1U);
  const casadi::MX terminal_yaw_delta = variables(state_index(config.horizon, 2U)) -
    parameters(terminal_reference + 2U);
  const casadi::MX terminal_wrapped_yaw = casadi::MX::atan2(
    casadi::MX::sin(terminal_yaw_delta), casadi::MX::cos(terminal_yaw_delta));
  objective += config.terminal_weight * (
    terminal_dx * terminal_dx + terminal_dy * terminal_dy +
    0.02 * terminal_wrapped_yaw * terminal_wrapped_yaw);

  for (std::size_t obstacle_index = 0; obstacle_index < slots; ++obstacle_index) {
    const std::size_t active_index = parameter_obstacle_base(config) +
      obstacle_index * obstacle_parameter_stride(config);
    const casadi::MX active = parameters(active_index);
    casadi::MX previous_clearance = symbolic_clearance(
      config, variables(state_index(0U, 0U)), variables(state_index(0U, 1U)),
      parameters, obstacle_index, 0U);
    append_constraint(active * previous_clearance + (1.0 - active), 0.0, casadi::inf);
    for (std::size_t k = 0; k < config.horizon; ++k) {
      const casadi::MX next_clearance = symbolic_clearance(
        config, variables(state_index(k + 1U, 0U)), variables(state_index(k + 1U, 1U)),
        parameters, obstacle_index, k + 1U);
      append_constraint(active * next_clearance + (1.0 - active), 0.0, casadi::inf);
      const casadi::MX required_slack = casadi::MX::fmax(
        config.gamma * previous_clearance - next_clearance, 0.0);
      objective += active * config.slack_weight * required_slack * required_slack;
      previous_clearance = next_clearance;
    }
  }

  graph->lower_constraint = std::move(lower_constraint);
  graph->upper_constraint = std::move(upper_constraint);
  const casadi::MX all_constraints = casadi::MX::vertcat(constraints);
  const casadi::MXDict nlp{
    {"x", variables}, {"p", parameters}, {"f", objective}, {"g", all_constraints}};
  casadi::Dict options;
  options["print_time"] = false;
  options["error_on_fail"] = false;
  options["ipopt.print_level"] = 0;
  options["ipopt.sb"] = "yes";
  options["ipopt.max_iter"] = config.max_iterations;
  // controller_server owns costmap and executor threads in the same process.
  // IPOPT's process CPU timer therefore charges unrelated Nav2 work and can
  // expire well before the solve's wall budget.  The contract is a wall-clock
  // control deadline, so use IPOPT 3.14's wall timer and retain the measured
  // post-solve rejection below.
  options["ipopt.max_wall_time"] = config.solver_budget_seconds;
  options["ipopt.tol"] = config.acceptable_tolerance;
  options["ipopt.acceptable_tol"] = config.acceptable_tolerance;
  options["ipopt.acceptable_obj_change_tol"] = config.acceptable_tolerance;
  options["ipopt.honor_original_bounds"] = "yes";

  const std::string name = "arena_mpc_" + std::to_string(graph_sequence.fetch_add(1U));
  graph->solver = casadi::nlpsol(name, "ipopt", nlp, options);
  graph->build_ms = milliseconds(build_start, Clock::now());
  return graph;
}

std::vector<double> initial_guess(const Config & config, const Problem & problem)
{
  std::vector<double> guess(control_base(config) + 2U * config.horizon, 0.0);
  State state = problem.initial_state;
  Control previous = problem.measured_control;
  for (std::size_t k = 0; k < config.horizon; ++k) {
    guess[state_index(k, 0U)] = state.x;
    guess[state_index(k, 1U)] = state.y;
    guess[state_index(k, 2U)] = state.yaw;
    const auto & target = problem.reference[k + 1U];
    const double dx = target.x - state.x;
    const double dy = target.y - state.y;
    const double distance = std::hypot(dx, dy);
    const double desired_yaw = distance > 1.0e-9 ? std::atan2(dy, dx) : target.yaw;
    const double interval = k == 0U ? problem.first_interval : config.dt;
    Control control;
    control.linear = clamp(
      distance / config.dt,
      std::max(config.min_linear, previous.linear - config.max_linear_accel * interval),
      std::min(
        effective_linear_limit(config, problem),
        previous.linear + config.max_linear_accel * interval));
    control.angular = clamp(
      wrap_angle(desired_yaw - state.yaw) / config.dt,
      std::max(-config.max_angular, previous.angular - config.max_angular_accel * interval),
      std::min(config.max_angular, previous.angular + config.max_angular_accel * interval));
    guess[control_index(config, k, 0U)] = control.linear;
    guess[control_index(config, k, 1U)] = control.angular;
    state = step(state, control, config.dt);
    previous = control;
  }
  guess[state_index(config.horizon, 0U)] = state.x;
  guess[state_index(config.horizon, 1U)] = state.y;
  guess[state_index(config.horizon, 2U)] = state.yaw;
  return guess;
}

std::vector<double> warm_start_guess(
  const Config & config, const Problem & problem, const std::vector<double> & previous_solution)
{
  std::vector<double> guess = initial_guess(config, problem);
  State state = problem.initial_state;
  Control previous = problem.measured_control;
  for (std::size_t k = 0; k < config.horizon; ++k) {
    guess[state_index(k, 0U)] = state.x;
    guess[state_index(k, 1U)] = state.y;
    guess[state_index(k, 2U)] = state.yaw;
    const std::size_t source_step = std::min(k + 1U, config.horizon - 1U);
    const double interval = k == 0U ? problem.first_interval : config.dt;
    Control control;
    control.linear = clamp(
      previous_solution[control_index(config, source_step, 0U)],
      std::max(config.min_linear, previous.linear - config.max_linear_accel * interval),
      std::min(
        effective_linear_limit(config, problem),
        previous.linear + config.max_linear_accel * interval));
    control.angular = clamp(
      previous_solution[control_index(config, source_step, 1U)],
      std::max(-config.max_angular, previous.angular - config.max_angular_accel * interval),
      std::min(config.max_angular, previous.angular + config.max_angular_accel * interval));
    guess[control_index(config, k, 0U)] = control.linear;
    guess[control_index(config, k, 1U)] = control.angular;
    state = step(state, control, config.dt);
    previous = control;
  }
  guess[state_index(config.horizon, 0U)] = state.x;
  guess[state_index(config.horizon, 1U)] = state.y;
  guess[state_index(config.horizon, 2U)] = state.yaw;
  return guess;
}

Trajectory unpack_trajectory(
  const Config & config, const Problem & problem, const std::vector<double> & values)
{
  Trajectory trajectory;
  trajectory.states.reserve(config.horizon + 1U);
  trajectory.controls.reserve(config.horizon);
  for (std::size_t k = 0; k <= config.horizon; ++k) {
    trajectory.states.push_back(State{
        values[state_index(k, 0U)], values[state_index(k, 1U)], values[state_index(k, 2U)]});
  }
  for (std::size_t k = 0; k < config.horizon; ++k) {
    trajectory.controls.push_back(Control{
        values[control_index(config, k, 0U)], values[control_index(config, k, 1U)]});
  }
  trajectory.barrier_slack.reserve(problem.obstacles.size() * config.horizon);
  for (std::size_t obstacle_index = 0; obstacle_index < problem.obstacles.size(); ++obstacle_index) {
    double previous_clearance = ellipse_clearance(
      trajectory.states.front(), problem.obstacles[obstacle_index].samples.front(),
      config.safe_distance, config.min_axis);
    for (std::size_t k = 0; k < config.horizon; ++k) {
      const double next_clearance = ellipse_clearance(
        trajectory.states[k + 1U], problem.obstacles[obstacle_index].samples[k + 1U],
        config.safe_distance, config.min_axis);
      trajectory.barrier_slack.push_back(
        std::max(0.0, config.gamma * previous_clearance - next_clearance));
      previous_clearance = next_clearance;
    }
  }
  return trajectory;
}

bool postcheck_passes(const Evaluation & evaluation, double tolerance)
{
  return evaluation.max_initial_residual <= tolerance &&
         evaluation.max_dynamics_residual <= tolerance &&
         evaluation.max_geometry_violation <= tolerance &&
         evaluation.max_barrier_violation <= tolerance &&
         evaluation.max_acceleration_violation <= tolerance &&
         evaluation.max_bound_violation <= tolerance && std::isfinite(evaluation.objective);
}

Problem select_relevant_obstacles(const Config & config, const Problem & problem)
{
  Problem selected = problem;
  selected.obstacles.clear();
  std::size_t selected_dynamic = 0U;
  for (const auto & obstacle : problem.obstacles) {
    if (!obstacle.dynamic) {
      continue;
    }
    bool reachable = false;
    for (std::size_t k = 0; k <= config.horizon; ++k) {
      const auto & sample = obstacle.samples[k];
      const double center_distance = std::hypot(
        sample.x - problem.initial_state.x, sample.y - problem.initial_state.y);
      const double obstacle_extent = std::max(sample.semi_major, sample.semi_minor) +
        config.safe_distance;
      const double reachable_distance =
        static_cast<double>(k) * config.dt * effective_linear_limit(config, problem);
      if (center_distance - obstacle_extent <= reachable_distance + config.acceptable_tolerance) {
        reachable = true;
        break;
      }
    }
    if (!reachable) {
      continue;
    }
    std::size_t & count = selected_dynamic;
    const std::size_t limit = config.max_nlp_dynamic_obstacles;
    if (count >= limit) {
      throw std::length_error("relevant dynamic NLP capacity exceeded");
    }
    ++count;
    selected.obstacles.push_back(obstacle);
  }
  return selected;
}

}  // namespace

class Solver::Impl
{
public:
  explicit Impl(Config config)
  : config_(std::move(config))
  {
    if (config_.horizon == 0U) {
      throw std::invalid_argument("horizon must be positive");
    }
    if (config_.layout == SolverLayout::FixedMasked) {
      fixed_graph_ = build_graph(
        config_, config_.max_nlp_dynamic_obstacles + config_.max_nlp_static_obstacles);
    }
  }

  Result solve(const Problem & problem, bool use_warm_start)
  {
    Result result;
    try {
      validate_problem(config_, problem);
    } catch (const std::length_error & error) {
      result.code = SolveCode::CapacityExceeded;
      result.status = error.what();
      return result;
    } catch (const std::exception & error) {
      result.code = SolveCode::InvalidInput;
      result.status = error.what();
      return result;
    }

    Problem selected_problem;
    try {
      selected_problem = select_relevant_obstacles(config_, problem);
    } catch (const std::length_error & error) {
      result.code = SolveCode::CapacityExceeded;
      result.status = error.what();
      return result;
    }

    const std::size_t capacity =
      config_.max_nlp_dynamic_obstacles + config_.max_nlp_static_obstacles;
    const std::size_t slots = config_.layout == SolverLayout::FixedMasked ?
      capacity : selected_problem.obstacles.size();
    Graph & graph = graph_for(slots);
    result.timing.graph_build_ms = graph.build_ms;

    const auto parameter_start = Clock::now();
    std::vector<double> parameters(graph.parameter_count, 0.0);
    parameters[0U] = problem.initial_state.x;
    parameters[1U] = problem.initial_state.y;
    parameters[2U] = problem.initial_state.yaw;
    parameters[3U] = problem.measured_control.linear;
    parameters[4U] = problem.measured_control.angular;
    parameters[5U] = problem.first_interval;
    for (std::size_t k = 0; k <= config_.horizon; ++k) {
      const std::size_t index = parameter_reference_base() + 3U * k;
      parameters[index] = problem.reference[k].x;
      parameters[index + 1U] = problem.reference[k].y;
      parameters[index + 2U] = problem.reference[k].yaw;
    }

    for (std::size_t obstacle_index = 0; obstacle_index < slots; ++obstacle_index) {
      const bool active = obstacle_index < selected_problem.obstacles.size();
      const std::size_t active_index = parameter_obstacle_base(config_) +
        obstacle_index * obstacle_parameter_stride(config_);
      parameters[active_index] = active ? 1.0 : 0.0;
      for (std::size_t k = 0; k <= config_.horizon; ++k) {
        const Ellipse placeholder{1000.0, 1000.0, 0.3, 0.3, 0.0};
        const Ellipse & sample = active ?
          selected_problem.obstacles[obstacle_index].samples[k] : placeholder;
        const std::size_t index = obstacle_parameter_index(config_, obstacle_index, k, 0U);
        parameters[index] = sample.x;
        parameters[index + 1U] = sample.y;
        parameters[index + 2U] = sample.semi_major;
        parameters[index + 3U] = sample.semi_minor;
        parameters[index + 4U] = sample.yaw;
      }
    }

    std::vector<double> lower_constraint = graph.lower_constraint;
    std::vector<double> upper_constraint = graph.upper_constraint;
    std::vector<double> upper_variable = graph.upper_variable;
    for (std::size_t k = 0; k < config_.horizon; ++k) {
      upper_variable[control_index(config_, k, 0U)] =
        effective_linear_limit(config_, problem);
    }
    lower_constraint[graph.first_linear_acceleration_constraint] =
      -config_.max_linear_accel * problem.first_interval;
    upper_constraint[graph.first_linear_acceleration_constraint] =
      config_.max_linear_accel * problem.first_interval;
    lower_constraint[graph.first_angular_acceleration_constraint] =
      -config_.max_angular_accel * problem.first_interval;
    upper_constraint[graph.first_angular_acceleration_constraint] =
      config_.max_angular_accel * problem.first_interval;

    const std::vector<double> guess =
      use_warm_start && graph.last_solution.size() == graph.variable_count ?
      warm_start_guess(config_, problem, graph.last_solution) : initial_guess(config_, problem);
    result.timing.parameter_update_ms = milliseconds(parameter_start, Clock::now());

    try {
      const casadi::DMDict arguments{
        {"x0", casadi::DM(guess)},
        {"p", casadi::DM(parameters)},
        {"lbx", casadi::DM(graph.lower_variable)},
        {"ubx", casadi::DM(upper_variable)},
        {"lbg", casadi::DM(lower_constraint)},
        {"ubg", casadi::DM(upper_constraint)}};
      const auto solve_start = Clock::now();
      const casadi::DMDict output = graph.solver(arguments);
      const auto solve_end = Clock::now();
      result.timing.solve_ms = milliseconds(solve_start, solve_end);
      const casadi::Dict stats = graph.solver.stats();
      result.status = stat_string(stats, "return_status");
      result.iterations = stat_int(stats, "iter_count");

      if (!stat_bool(stats, "success")) {
        result.code = result.status.find("Time") != std::string::npos ?
          SolveCode::Timeout : SolveCode::Infeasible;
        return result;
      }
      if (result.timing.solve_ms > config_.solver_budget_seconds * 1000.0) {
        result.code = SolveCode::Timeout;
        result.status += ": measured solver budget exceeded";
        return result;
      }

      const std::vector<double> values = output.at("x").nonzeros();
      if (values.size() != graph.variable_count) {
        result.code = SolveCode::SolverError;
        result.status = "solver returned an unexpected decision vector size";
        return result;
      }
      result.trajectory = unpack_trajectory(config_, problem, values);
      const auto postcheck_start = Clock::now();
      result.evaluation = evaluate(config_, problem, result.trajectory);
      result.timing.postcheck_ms = milliseconds(postcheck_start, Clock::now());
      if (!postcheck_passes(result.evaluation, config_.acceptable_tolerance)) {
        result.code = SolveCode::PostcheckFailed;
        result.status += ": independent postcheck failed";
        return result;
      }

      graph.last_solution = values;
      result.code = SolveCode::Success;
      result.command_valid = true;
      return result;
    } catch (const std::exception & error) {
      result.code = SolveCode::SolverError;
      result.status = error.what();
      return result;
    }
  }

  const Config & config() const noexcept
  {
    return config_;
  }

  void reset()
  {
    for (auto & item : exact_graphs_) {
      item.second->last_solution.clear();
    }
    if (fixed_graph_) {
      fixed_graph_->last_solution.clear();
    }
  }

private:
  Graph & graph_for(std::size_t slots)
  {
    if (config_.layout == SolverLayout::FixedMasked) {
      if (!fixed_graph_) {
        fixed_graph_ = build_graph(config_, slots);
      }
      return *fixed_graph_;
    }
    const auto found = exact_graphs_.find(slots);
    if (found != exact_graphs_.end()) {
      return *found->second;
    }
    auto graph = build_graph(config_, slots);
    Graph * graph_pointer = graph.get();
    exact_graphs_.emplace(slots, std::move(graph));
    return *graph_pointer;
  }

  Config config_;
  std::unordered_map<std::size_t, std::unique_ptr<Graph>> exact_graphs_;
  std::unique_ptr<Graph> fixed_graph_;
};

Solver::Solver(Config config)
: impl_(std::make_unique<Impl>(std::move(config)))
{
}

Solver::~Solver() = default;
Solver::Solver(Solver &&) noexcept = default;
Solver & Solver::operator=(Solver &&) noexcept = default;

Result Solver::solve(const Problem & problem, bool use_warm_start)
{
  return impl_->solve(problem, use_warm_start);
}

const Config & Solver::config() const noexcept
{
  return impl_->config();
}

void Solver::reset()
{
  impl_->reset();
}

}  // namespace arena_mpc_core

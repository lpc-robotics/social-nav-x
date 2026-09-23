#ifndef ARENA_MPC_CORE__TYPES_HPP_
#define ARENA_MPC_CORE__TYPES_HPP_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#if defined _WIN32 || defined __CYGWIN__
  #ifdef ARENA_MPC_CORE_BUILDING_DLL
    #define ARENA_MPC_CORE_PUBLIC __declspec(dllexport)
  #else
    #define ARENA_MPC_CORE_PUBLIC __declspec(dllimport)
  #endif
#else
  #define ARENA_MPC_CORE_PUBLIC __attribute__((visibility("default")))
#endif

namespace arena_mpc_core
{

struct State
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Control
{
  double linear{0.0};
  double angular{0.0};
};

struct Ellipse
{
  double x{0.0};
  double y{0.0};
  double semi_major{0.3};
  double semi_minor{0.3};
  double yaw{0.0};
};

struct ObstaclePrediction
{
  std::uint64_t id{0};
  bool dynamic{false};
  std::vector<Ellipse> samples;
};

enum class SolverLayout
{
  ExactActive,
  FixedMasked
};

struct Config
{
  std::size_t horizon{25};
  double dt{0.1};
  double gamma{0.2};
  double safe_distance{0.3};
  double max_initial_clearance_violation{0.05};
  double terminal_weight{1.0};
  double slack_weight{50.0};
  double min_linear{0.0};
  double max_linear{0.8};
  double max_angular{1.5};
  double max_linear_accel{2.0};
  double max_angular_accel{3.2};
  double min_axis{1.0e-3};
  std::size_t max_dynamic_obstacles{32};
  std::size_t max_static_obstacles{128};
  std::size_t max_nlp_dynamic_obstacles{8};
  std::size_t max_nlp_static_obstacles{0};
  int max_iterations{100};
  double solver_budget_seconds{0.075};
  double acceptable_tolerance{1.0e-3};
  SolverLayout layout{SolverLayout::FixedMasked};
};

struct Problem
{
  State initial_state;
  Control measured_control;
  double first_interval{0.1};
  double linear_speed_limit{0.0};
  std::vector<State> reference;
  std::vector<ObstaclePrediction> obstacles;
};

struct Trajectory
{
  std::vector<State> states;
  std::vector<Control> controls;
  std::vector<double> barrier_slack;
};

struct Evaluation
{
  double objective{0.0};
  double max_initial_residual{0.0};
  double max_dynamics_residual{0.0};
  double max_geometry_violation{0.0};
  double max_barrier_violation{0.0};
  double max_acceleration_violation{0.0};
  double max_bound_violation{0.0};
};

struct Timing
{
  double graph_build_ms{0.0};
  double parameter_update_ms{0.0};
  double solve_ms{0.0};
  double postcheck_ms{0.0};
};

enum class SolveCode
{
  Success,
  InvalidInput,
  CapacityExceeded,
  Infeasible,
  Timeout,
  SolverError,
  PostcheckFailed
};

struct Result
{
  SolveCode code{SolveCode::SolverError};
  std::string status;
  Trajectory trajectory;
  Evaluation evaluation;
  Timing timing;
  int iterations{0};
  bool command_valid{false};
  bool accepted_nonoptimal_iterate{false};
};

}  // namespace arena_mpc_core

#endif  // ARENA_MPC_CORE__TYPES_HPP_

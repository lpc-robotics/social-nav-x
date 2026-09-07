#ifndef ARENA_MPC_CORE__MODEL_HPP_
#define ARENA_MPC_CORE__MODEL_HPP_

#include "arena_mpc_core/types.hpp"

namespace arena_mpc_core
{

ARENA_MPC_CORE_PUBLIC double wrap_angle(double angle);
ARENA_MPC_CORE_PUBLIC State step(const State & state, const Control & control, double dt);
ARENA_MPC_CORE_PUBLIC double ellipse_clearance(
  const State & state, const Ellipse & obstacle, double safe_distance, double min_axis);
ARENA_MPC_CORE_PUBLIC void validate_problem(const Config & config, const Problem & problem);
ARENA_MPC_CORE_PUBLIC Evaluation evaluate(
  const Config & config, const Problem & problem, const Trajectory & trajectory);

}  // namespace arena_mpc_core

#endif  // ARENA_MPC_CORE__MODEL_HPP_

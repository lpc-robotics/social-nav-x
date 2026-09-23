#ifndef ARENA_MPC_CONTROLLER__REFERENCE_BUILDER_HPP_
#define ARENA_MPC_CONTROLLER__REFERENCE_BUILDER_HPP_

#include <cstddef>
#include <vector>

#include <arena_mpc_core/types.hpp>

namespace arena_mpc_controller
{

struct ReferenceBuildResult
{
  std::vector<arena_mpc_core::State> states;
  bool position_latched{false};
  double goal_distance{0.0};
};

ReferenceBuildResult build_path_reference(
  const std::vector<arena_mpc_core::State> & path,
  const arena_mpc_core::State & robot,
  std::size_t horizon,
  double spacing,
  double xy_goal_tolerance,
  bool position_already_latched);

}  // namespace arena_mpc_controller

#endif  // ARENA_MPC_CONTROLLER__REFERENCE_BUILDER_HPP_

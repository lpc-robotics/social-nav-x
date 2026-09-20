#ifndef ARENA_MPC_CONTROLLER__CLEARANCE_RECOVERY_HPP_
#define ARENA_MPC_CONTROLLER__CLEARANCE_RECOVERY_HPP_

#include <cstdint>
#include <vector>

#include <arena_mpc_core/types.hpp>
#include <geometry_msgs/msg/point.hpp>

namespace arena_mpc_controller
{

struct ClearanceRecoveryDecision
{
  bool active{false};
  bool admissible{false};
  arena_mpc_core::Control control;
  double minimum_circle_clearance{0.0};
  double heading_error{0.0};
  std::uint64_t limiting_obstacle_id{0U};
};

ClearanceRecoveryDecision make_clearance_recovery_decision(
  const arena_mpc_core::Config & config,
  const arena_mpc_core::Problem & problem,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  double robot_circumscribed_radius,
  double exit_clearance,
  double desired_linear,
  double maximum_angular,
  double minimum_outward_cosine,
  bool recovery_was_active);

}  // namespace arena_mpc_controller

#endif  // ARENA_MPC_CONTROLLER__CLEARANCE_RECOVERY_HPP_

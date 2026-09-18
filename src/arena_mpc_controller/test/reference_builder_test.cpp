#include "arena_mpc_controller/reference_builder.hpp"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace
{

bool require(bool condition, const std::string & message)
{
  if (!condition) {
    std::cerr << "REFERENCE_BUILDER_TEST_FAILED " << message << '\n';
  }
  return condition;
}

}  // namespace

int main()
{
  const std::vector<arena_mpc_core::State> westbound_path{
    {4.0, 3.0, M_PI},
    {3.5, 3.0, M_PI},
    {3.0, 3.0, 0.0}};

  const auto outside = arena_mpc_controller::build_path_reference(
    westbound_path, {3.190052, 2.822809, M_PI}, 25U, 0.025, 0.25, false);
  if (!require(!outside.position_latched, "0.260 m pose was treated as position reached") ||
    !require(outside.states.size() == 26U, "wrong horizon size") ||
    !require(
      std::abs(std::abs(outside.states.back().yaw) - M_PI) < 1.0e-9,
      "terminal samples requested final goal yaw before entering XY tolerance"))
  {
    return 1;
  }

  const auto inside = arena_mpc_controller::build_path_reference(
    westbound_path, {3.20, 3.0, M_PI}, 25U, 0.025, 0.25, false);
  if (!require(inside.position_latched, "pose inside XY tolerance did not latch") ||
    !require(std::abs(inside.states.front().x - 3.0) < 1.0e-12, "latched X is not goal X") ||
    !require(std::abs(inside.states.front().y - 3.0) < 1.0e-12, "latched Y is not goal Y") ||
    !require(std::abs(inside.states.front().yaw) < 1.0e-12, "latched yaw is not goal yaw"))
  {
    return 2;
  }

  const auto retained = arena_mpc_controller::build_path_reference(
    westbound_path, {3.30, 3.0, M_PI}, 25U, 0.025, 0.25, true);
  if (!require(retained.position_latched, "position latch was lost after small drift") ||
    !require(std::abs(retained.states.back().yaw) < 1.0e-12, "latched final yaw changed"))
  {
    return 3;
  }

  std::cout << "REFERENCE_BUILDER_TEST_OK goal_distance=" << outside.goal_distance << '\n';
  return 0;
}

#include <arena_mpc_core/model.hpp>

#include <cmath>
#include <iostream>

int main()
{
  const arena_mpc_core::State initial{1.0, 2.0, 0.0};
  const arena_mpc_core::Control command{0.2, 0.1};
  const auto next = arena_mpc_core::step(initial, command, 0.1);
  if (std::abs(next.x - 1.02) > 1.0e-12 || std::abs(next.y - 2.0) > 1.0e-12 ||
    std::abs(next.yaw - 0.01) > 1.0e-12)
  {
    return 1;
  }
  std::cout << "ARENA_MPC_CORE_CONSUMER_OK\n";
  return 0;
}

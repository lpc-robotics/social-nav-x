#ifndef ARENA_MPC_CORE__SOLVER_HPP_
#define ARENA_MPC_CORE__SOLVER_HPP_

#include <memory>

#include "arena_mpc_core/types.hpp"

namespace arena_mpc_core
{

class ARENA_MPC_CORE_PUBLIC Solver
{
public:
  explicit Solver(Config config);
  ~Solver();
  Solver(Solver &&) noexcept;
  Solver & operator=(Solver &&) noexcept;
  Solver(const Solver &) = delete;
  Solver & operator=(const Solver &) = delete;

  Result solve(const Problem & problem, bool use_warm_start = true);
  const Config & config() const noexcept;
  void reset();

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace arena_mpc_core

#endif  // ARENA_MPC_CORE__SOLVER_HPP_

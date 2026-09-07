# P1 mathematical core validation

Date: 2026-09-07 (Asia/Shanghai)

## Implemented boundary

`arena_mpc_core` is a ROS-independent C++17 library. It implements:

- unicycle dynamics with `N=25` and `dt=0.1 s`;
- wrapped-yaw reference tracking, control cost, and a corrected `x[N]`
  terminal cost;
- odometry-anchored first-control acceleration limits and adjacent-control
  limits thereafter;
- rotated-ellipse clearance with finite behavior at the ellipse center;
- hard geometry constraints and the same soft barrier relation, with the
  nonnegative quadratic slack analytically eliminated;
- fixed eight-slot dynamic-obstacle NLP with finite inactive parameters and an
  active mask;
- input capacity of 32 dynamic and 128 static objects, conservative dynamic
  reachability selection, full-input trajectory postcheck, and fail-closed
  capacity handling.

Static costmap geometry is intentionally kept out of the ellipse NLP. It is an
input to the independent trajectory and braking-trajectory collision checks in
the controller layer. This follows the actual `lidar -> costmap -> controller`
chain and avoids representing inflated costmap cells as physical ellipses.

## Numerical tests

The package was built with the target RoboStack GCC 15.3 compiler, C++17, and
`-Wall -Wextra -Wpedantic -Werror`. All four CTest cases pass.

```text
PYTHON_REFERENCE_OK worst_difference=4.92661467177e-16
PYTHON_SOLVER_REFERENCE_OK command_difference=0 objective_difference=1.38777878078e-17
SOLVER_CASES_OK cold_ms=28.5885 warm_ms=19.5151 first_linear=0.179484
```

The defect cases cover wrapped angles across `+/-pi`, terminal-state indexing,
horizontal and vertical ellipses, center distance zero, degenerate axes,
initial overlap, input-capacity overflow, relevant-NLP-capacity overflow, empty
obstacles, and first-control acceleration anchored to measured odometry.

The installed package exports `arena_mpc_core::arena_mpc_core`; an external
CMake consumer compiled and ran as follows:

```text
ARENA_MPC_CORE_CONSUMER_OK
```

`ldd -r` found no unresolved symbols when the isolated CasADi runtime closure
was supplied. The core library has `$ORIGIN` plus the frozen RoboStack underlay
in its install RPATH and does not reference the development CasADi prefix.

## Gate result

The P1 numerical thresholds pass: evaluator difference is below `1e-6`, the
fixed feasible first control difference is below `1e-3`, and successful NLP
residuals are below `1e-3`. Failed solves never set `command_valid` and never
return a shifted old command.

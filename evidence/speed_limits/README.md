# 0.8 m/s and 1.5 rad/s speed-limit validation

Date: 2026-09-17

Release under test: `20260916-4ff2edd`, source commit
`4ff2edd9715e2ee839f055558450dc0297886927`.

## Configuration and runtime contract

- MPC: `FollowPath.max_linear=0.8`, `FollowPath.max_angular=1.5`;
  velocity smoother max/min are `[0.8, 0.0, 1.5]` and
  `[-0.8, 0.0, -1.5]`.
- DWB additive profile: `FollowPath.max_vel_x=0.8`,
  `FollowPath.max_speed_xy=0.8`, `FollowPath.max_vel_theta=1.5`; velocity
  smoother max/min are `[0.8, 0.0, 1.5]` and `[-0.8, 0.0, -1.5]`.
- The MPC reference spacing remains `0.025 m` per `0.1 s` model step. The
  upper bound changed; the nominal path reference was not forced to 0.8 m/s.
- The original stable `scripts/run_six_behaviors.sh` remains the default DWB
  entry. Its SHA-256 is
  `9f8d26eae21348e00e6141f95829e51bf5cbc21e0c57d541bd3c8e9df2530836`.
  The 0.8/1.5 DWB profile uses the additive
  `scripts/run_six_behaviors_dwb_08.sh` entry.

The parameter-contract tests passed 3/3. The complete stored colcon result is
18 tests, 0 errors, 0 failures, and 0 skipped.

## Accepted live runs

Both runs used the immutable release, the same six-behavior HuNav
configuration (SHA-256
`ac77047d69256d71fadca0aa958848cb9b30a0b15e3fcf91275514fc1a748b67`),
ideal chassis mode, a 1/60 s physics step, and GPU 1. Each action status is 4
(`SUCCEEDED`) and each JSON has `pass=true`.

| Metric | MPC | DWB 0.8/1.5 profile |
|---|---:|---:|
| ROS domain | 223 | 225 |
| Configured linear limit | 0.8 m/s | 0.8 m/s |
| Command samples | 763 | 104 |
| Maximum observed linear command | 0.2600 m/s | 0.3789 m/s |
| Maximum observed angular command | 0.1050 rad/s | 0.0789 rad/s |
| Final position error | 0.2275 m | 0.2166 m |
| Measured minimum footprint-human clearance | 0.5767 m | 1.1501 m |
| Sampling alignment error bound | 0.05661 m | 0.05610 m |
| Conservative clearance lower bound | 0.5201 m | 1.0940 m |

Raw structured results are `mpc_live.json` and `dwb_live.json`; the associated
launch logs show the selected plugins, lifecycle activation, HuNav readiness,
and orderly shutdown.

## Validation correction

The comparison probe previously required a sampling-alignment error bound no
larger than 0.05 m. Its bound uses the configured maximum robot speed, so the
same 25 ms human-stamp gap and 16.7 ms odom bracket produce approximately
0.0566 m after the speed limit increase. The audit ceiling is now 0.06 m. The
hard conservative human-clearance requirement remains 0.30 m and still
subtracts the computed alignment error from measured clearance.

The probe's MPC global-costmap expectation was also updated from the obsolete
`static + obstacle + inflation` list to the released
`static + inflation` contract. This follows the independently validated global
costmap correction and does not change runtime configuration.

Release verification passed for all 260 files (45,628,566 bytes): full
SHA-256 manifest, zero symlinks, both MPC and DWB runtime-only checks, stable
underlay protection, and dependency resolution.

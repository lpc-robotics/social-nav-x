# P6 immutable release and rollback gate

The latest P6 corrective release passed on 2026-09-18. It published the
additive release `20260918-bd63612`, exercised that exact release through the
stable-workspace MPC entry, and then exercised the original protected DWB
entry in a fresh ROS domain. Earlier immutable releases remain available.

The 45,964,467-byte release is at
`/home/lpc/workspace/arena5_ws/optional/mpc/releases/20260918-bd63612`. Its
`RELEASE` file binds controller/configuration hashes to source commit
`bd63612de9ca5ae28986a063257a2bf4a37c073a`. `SHA256SUMS` covers every file.
The release has no symlinks, development path, staging path, or unresolved
runtime dependency after loading the Arena underlay.

The candidate was built with tests disabled in a temporary prefix, moved to a
different temporary path, and passed runtime, benchmark, dependency, and path
checks before the stable workspace was touched. Publication added the new
immutable directory and atomically selected it through the independent MPC and
DWB08 wrappers. The original `run_six_behaviors.sh` remains unchanged at SHA-256
`9f8d26eae21348e00e6141f95829e51bf5cbc21e0c57d541bd3c8e9df2530836`.

The final development build passed all 20 model, solver, reference, plugin,
progress, terminal-reference, visualization, costmap, and configuration tests.
The published checksum manifest, runtime-only check, three binary dependency
checks, zero-path scan, and protected stable manifest all passed.

`formal_summary.json` records the release smoke:

| Entry | Runtime controller | Configured linear limit | Action | Position error | Clearance lower bound |
|---|---|---:|---:|---:|---:|
| released MPC | `arena_mpc_controller::MpcController` | 0.8 m/s | success | 0.2254 m | 0.5224 m |
| original DWB | `dwb_core::DWBLocalPlanner` | 0.26 m/s | success | 0.2265 m | 0.4869 m |

The original DWB limit is intentionally unchanged; the separate DWB08 wrapper
provides the 0.8/1.5 profile. Both smoke results used GPU 3, 1/60 s physics,
the same six-behavior input, and independent domains 214 and 215. Commands were
finite and each result met the 0.25 m goal tolerance.

Launch logs remain local and are ignored by Git. The accepted JSON files bind
controller identity, effective geometry, safety alignment bounds, GPU/domain
metadata, input hashes, action result, and motion metrics.

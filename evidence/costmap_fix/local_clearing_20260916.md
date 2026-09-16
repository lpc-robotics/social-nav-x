# Local costmap stale-obstacle abort diagnosis and correction

## Live reproduction

The failure was reproduced online on 2026-09-16 with release
`20260915-bf2bc7c`. A goal at `(11.297, 3.499)` was accepted at 16:36:50. Navfn
continued to return valid global paths while the robot moved from `(3.000,
3.000)` to `(3.688, 3.002)`.

The MPC status first entered `human_wait` while a pedestrian crossed the
trajectory. After that pedestrian moved away, it changed to
`costmap_postcheck`. At 16:37:09 the predicted full-footprint check repeatedly
failed at `(4.1619, 3.00064, -0.00363)`. The one-second bounded costmap wait
expired, five consecutive controller failures accumulated, `FollowPath`
aborted, and `NavigateToPose` became `ABORTED` at 16:37:10. This was an MPC
local-collision rejection, not a global-planner no-path result and not the
progress checker.

The post-failure local costmap contained lethal cells beginning near
`(4.45, 3.25)`, inside the Jackal's predicted front-footprint sweep. The
corresponding current RTX LaserScan rays were `-1` (and some were `0`), so they
could not form VoxelLayer clearing rays with `inf_is_valid=false`. No current
human occupied the rejected point. The stale trail therefore remained after
the moving human had left and directly caused the abort.

## Correction

Commit `832939866ee8809169e31ae446fe3da0e07d62d5` adds a release-local Isaac
runtime overlay based on the already validated depth-clearing implementation
at `be8fefce4fdeb238372da923214e478d92bbbc32`.

- `/lidar` is unchanged and remains the local obstacle-marking source.
- Four co-located 90-degree render-depth views publish `/lidar_clearing`.
- The PointCloud2 source is configured `marking=false`, `clearing=true`, with a
  `3.0 m` raytrace limit and zero observation persistence.
- Finite rays stop `0.05 m` before rendered surfaces. Only positive finite
  depth and explicit positive infinity produce endpoints; negative, zero, NaN,
  and negative infinity remain unobserved.
- The global costmap remains `static_layer + inflation_layer`.
- `ARENA_DEPTH_CLEARING=false` disables the added render products and publisher
  at the next launch.

The stable Arena source/install tree was not edited. The complete patched
Python runtime is carried inside the immutable MPC release and prepended to
`PYTHONPATH` only by that release's runner.

## Build, release, and rollback checks

- Development build: three packages succeeded.
- Regression result: 16 tests, zero errors, failures, or skips.
- Immutable release: `20260916-8329398`, 45,298,093 bytes, 207 SHA-256 entries.
- Release checks: relocation passed, no symlinks, no development/staging paths,
  no unresolved native dependencies, and all seven protected-underlay hashes
  matched.
- Default-enabled and `ARENA_DEPTH_CLEARING=false` runtime checks both passed.
- Atomic rollback drill passed in both directions:
  `20260916-8329398 -> 20260915-bf2bc7c -> 20260916-8329398`.

The currently running pre-fix process is intentionally untouched. Final
task-level acceptance still requires restarting through the stable wrapper and
repeating the same navigation while checking `/lidar_clearing`, local lethal
cell removal, action completion, and command continuity.

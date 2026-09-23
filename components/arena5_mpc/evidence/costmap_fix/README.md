# Global costmap obstacle-layer correction

This correction was published on 2026-09-15 as immutable release
`20260915-bf2bc7c`, bound by its `RELEASE` metadata to source commit
`bf2bc7c31ec1dadd51318b20485eab138744c452`.

## Decision

The MPC-mode global costmap now declares exactly `static_layer` and
`inflation_layer`. The global `obstacle_layer` override and its lidar timing
override were removed. This prevents raw RTX lidar returns, whose invalid
samples do not provide reliable clearing rays, from persisting as lethal
global cells and causing a false Navfn no-path result.

The local avoidance path is intentionally unchanged. The local costmap retains
its lidar-backed `voxel_layer`, including the `0.3 s` expected update rate;
the command watchdog still monitors `/local_costmap/costmap_raw`; and dynamic
people remain direct `/human_states` inputs to the MPC controller. Removing the
global layer therefore does not remove MPC's local trajectory or braking
collision checks.

Re-enable a global lidar obstacle layer only if the static map is known to omit
obstacles that require global topological detours and the selected observation
source supplies reliable clearing (or an equivalent bounded-lifetime filter).

## Verification and rollback

- The three-package development build completed successfully.
- The full result was `12 tests, 0 errors, 0 failures, 0 skipped`, including a
  new configuration contract that fixes the global layer list and verifies the
  local lidar/watchdog path.
- The release passed relocation, dependency, development-path, SHA-256, and all
  seven protected-underlay checks. The installed release configuration reports
  `static_layer,inflation_layer`, no global obstacle override, and local lidar
  expected update rate `0.3`.
- The stable wrapper was switched from the new release to
  `20260915-df9a55d` and back to `20260915-bf2bc7c`. Both directions passed
  checksum, protected-underlay, and package-resolution checks.

The selector validates the target before atomically replacing the wrapper,
refuses an unrecognized hand-edited wrapper, and restores the previous wrapper
if the post-switch runtime check fails. Selection affects only a future launch.

Rollback command:

```bash
cd /home/lpc/workspace/arena5_mpc_ws
./scripts/select_mpc_release.sh 20260915-df9a55d
```

Return to the corrected release with:

```bash
./scripts/select_mpc_release.sh 20260915-bf2bc7c
```

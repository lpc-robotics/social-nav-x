# social-nav-x

Source archive of the validated Arena-Rosnav 5.0 + Isaac Sim 5.1 social
navigation workspace, including the ideal D6 velocity-controlled Jackal,
HuNav six behaviors, Nav2, WebRTC, Foxglove, odometry, TF, and sensor bridge.

This snapshot was prepared from `/home/lpc/workspace/arena5_ws` on
2026-08-29. It preserves the project-owned source and every local upstream
change without committing the 20+ GB Conda/build/cache/log workspace.

## What is stored here

- `src/arena-isaac/`: complete flattened Arena Isaac source with all current
  Humble, Isaac 5.1, HuNav, odometry, and D6 modifications.
- `src/arena-rosnav/`: complete flattened Arena-Rosnav source with the current
  six-behavior launch and configuration.
- `scripts/` and `config/`: current build, launch, cleanup, isolated matrix,
  and collision-test tooling plus the generated Jackal URDF reference.
- `patches/`: current binary-safe diffs against every modified upstream base.
- `upstream/manifest.tsv`: exact origin URL, commit, branch, storage method,
  and patch for all nested repositories.
- `evidence/`: compact final velocity, collision, system-regression, screenshot,
  and completion-audit artifacts.
- `HANDOFF.md`, `DEPLOYMENT.md`, and `CHASSIS_CONTROL.md`: authoritative
  operating, architecture, validation, and recovery documentation.

Large unmodified upstream repositories are pinned rather than copied. Restore
them into the original workspace layout with:

```bash
./scripts/fetch_upstreams.sh
```

The script checks out exact commits and applies the archived local patches for
simulation-setup, HuNavSim, and Foxglove. It never resets an existing dirty
repository.

## Build and run

The deployment reuses Isaac Sim 5.1 from
`/home/lpc/miniforge3/envs/isaaclab` and the workspace-local ROS 2 Humble Conda
environment described in `DEPLOYMENT.md`. On the archived host layout:

```bash
source scripts/env.sh
./scripts/build.sh
GPU_ID=3 ./scripts/run_six_behaviors.sh
```

The ideal D6 chassis is enabled by default. Use
`ARENA_IDEAL_CHASSIS=false` only for an explicit comparison with the legacy
wheel-contact skid-steer fallback.

## Validated result

- isolated velocity matrix and hold-outs: 22/22 valid;
- maximum mean error: 0.086511% linear, 0.313368% angular;
- frontal, oblique, and combined-turn collision tests: 3/3 valid with no wall
  penetration;
- Nav2: `SMOKE_NAVIGATION_OK`, 1.811 m motion;
- HuNav: `SIX_BEHAVIORS_VERIFY_OK types=1,2,3,4,5,6`;
- WebRTC, Foxglove, odom, TF, lidar, point cloud, and joint states verified;
- final default instance stable for approximately 5 h 46 min and shut down
  cleanly.

See `CHASSIS_CONTROL.md` for complete before/after tables and exact evidence.

## Licensing

Vendored upstream source retains its original license files and notices. This
archive does not replace or relicense those components.

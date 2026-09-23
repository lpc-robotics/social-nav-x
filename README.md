# social-nav-x

Source archive of the validated Arena-Rosnav 5.0 + Isaac Sim 5.1 social
navigation workspace, including the ideal D6 velocity-controlled Jackal,
HuNav six behaviors, Nav2, WebRTC, Foxglove, odometry, TF, and sensor bridge.

The original snapshot was prepared from `/home/lpc/workspace/arena5_ws` on
2026-08-29. The repository was consolidated again on 2026-09-23 with the
subsequent formal-social, MPC, and accepted radar-input development histories.
It preserves project-owned source and local upstream changes without committing
the Conda, build, install, runtime-log, or binary-release workspaces.

## What is stored here

- `src/arena-isaac/`: complete flattened Arena Isaac source with all current
  Humble, Isaac 5.1, HuNav, odometry, and D6 modifications.
- `src/arena-rosnav/`: complete flattened Arena-Rosnav source with the current
  six-behavior launch and configuration.
- `src/formal_social_behavior/`: deterministic single- and multi-human formal
  social automata, ROS proxies, configuration, replay, trace, and tests.
- `components/arena5_mpc/`: full history and source of the validated Arena5 MPC
  controller, Nav2 plugin, launch integration, tests, and compact evidence.
- `components/radar_input/`: accepted normalized-LaserScan and depth-clearing
  development history, contracts, validation reports, tools, and incremental
  source patches. These remain opt-in and do not change the stable default.
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

The script checks out exact commits and applies the archived stable patches for
simulation-setup, HuNavSim, and Foxglove. It never resets an existing dirty
repository. Optional radar-input patch order and exact source commits are in
`components/radar_input/SOURCE_PATCHES.md`.

## Consolidated development lines

The following previously local-only histories are reachable from `main`:

| Area | Imported tip | Location |
|---|---|---|
| Formal social automata Phase 4 | `0d9ec67` | `src/formal_social_behavior/` |
| Arena5 MPC and DWB comparison | `29edf7b` | `components/arena5_mpc/` |
| Accepted radar input | `6d2773b` | `components/radar_input/` |

The component imports retain their original commits as merge parents. Generated
MPC releases under `arena5_ws/optional`, colcon products, NvStreamer traces,
command transcripts, and runtime logs are intentionally excluded; all core MPC
packages needed to rebuild are stored under `components/arena5_mpc/src/`.

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

Formal-social and MPC development use isolated overlays. Their build and test
entry points are documented in `FORMAL_SOCIAL_AUTOMATA_DEVELOPMENT_PLAN.md` and
`components/arena5_mpc/README.md`. Do not copy the accepted normalized-radar
configuration over the stable default; use the isolated procedure in
`components/radar_input/RADAR_INPUT_HANDOFF.md`.

## Licensing

Vendored upstream source retains its original license files and notices. This
archive does not replace or relicense those components.

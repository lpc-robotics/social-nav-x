# Arena-Rosnav 5.0 + Isaac Sim 5.1 deployment

Validated through 2026-08-28 in `/home/lpc/workspace/arena5_ws`.

For the current continuation state, open `HANDOFF.md`. This document describes
the reproducible deployment, launch modes, architecture, and validation.
`CHASSIS_CONTROL.md` is the authoritative ideal-D6 chassis design, test, and
recovery record.

## Versions and host inventory

- Ubuntu 22.04.5 LTS, kernel 6.8.0-136-generic.
- 4 x NVIDIA GeForce RTX 4090 (24564 MiB each).
- NVIDIA driver 580.126.09; host CUDA toolkit 13.0 (V13.0.88).
- Conda 26.7.0 and Mamba 2.5.0 in `/home/lpc/miniforge3`.
- ROS 2 Humble (RoboStack `ros-humble-ros-base` 0.10.0) in the
  workspace-local environment `.conda/arena_ros`.
- Isaac Sim 5.1.0.0 reused from
  `/home/lpc/miniforge3/envs/isaaclab`.
- HuNavSim v1 (`hunav_agent_manager` 1.0.0).
- Foxglove Bridge 3.4.3, built locally for ROS 2 Humble.
- Docker is not installed and is not used.
- Free filesystem space at the latest check: approximately 265 GiB.
- No `sudo`, `apt`, `apt-get`, or `pip install --user` was used.

## Sources

- `src/arena-rosnav`: branch `humble`, commit
  `c2ff4a87e8686013b53f1e9cd8b01b3ab04fbce4` plus the six-behavior launch,
  config, package, and navigation changes.
- `src/arena-isaac`: branch `arena5-isaac5.1.0`, commit
  `16b8e3416517d8c3dc1b5038df4fe11b9a6df46c` plus the Humble/Isaac 5.1
  compatibility and six-behavior bridge changes.
- `src/arena/simulation-setup`: branch `humble-fix`, commit
  `3f142b25d88ce962c803b57cf20f38985d376dea` plus the Nav2 launch diff.
- `src/deps/hunav/hunav_sim`: detached at
  `a69cf96d98b0d40e247f819d7aebab661ac68b3b` plus two behavior corrections.
- `src/deps/foxglove-sdk`: detached at
  `05f27efc7e535d9c30c6b0cb4f6aa89de7243870` plus ROS 2 Humble compile
  compatibility.

These source trees contain intentional, uncommitted work. Patch records are in
`logs/upstream_diffs/`. Save or refresh them before pulling, rebasing, or
checking out upstream code. Do not discard the working trees.

## Environment

Always source the workspace wrapper before using ROS commands:

```bash
cd /home/lpc/workspace/arena5_ws
source scripts/env.sh
```

It configures:

- `ARENA_WS=/home/lpc/workspace/arena5_ws`.
- ROS 2 Humble, `ROS_DOMAIN_ID=51`, and `rmw_fastrtps_cpp`.
- Isaac's Python 3.11 modules and Humble ROS 2 Bridge libraries.
- Arena-local XDG, Conda, and pip cache directories.
- `LD_PRELOAD` and `LD_LIBRARY_PATH` needed by the mixed Conda/Isaac process.
- WebRTC TCP 49100 and UDP 47998.
- Foxglove WebSocket `127.0.0.1:8765`.

When `GPU_ID=3`, `CUDA_VISIBLE_DEVICES=3` isolates the process to host GPU 3.
CUDA code inside that process must use `cuda:0`; Kit/Vulkan receives host
render ordinal 3 through `ARENA_RENDER_GPU`.

## Build

Rebuild all workspace-owned packages and regenerate the Jackal URDF:

```bash
cd /home/lpc/workspace/arena5_ws
source scripts/env.sh
./scripts/build.sh
```

The build is split into dependency, Arena/Isaac/HuNav, and Foxglove phases so
that generated message types are sourced before dependent packages. It also
stages the lightsfm headers and writes
`config/generated/jackal.urdf` with workspace-local mesh paths.

For a change limited to the six-behavior launch:

```bash
colcon build --packages-select arena_bringup
source install/setup.bash
```

## Launch modes

### Six HuNav behaviors with navigation, WebRTC, and Foxglove

This is the current primary demo:

```bash
cd /home/lpc/workspace/arena5_ws
source scripts/env.sh
GPU_ID=3 ./scripts/run_six_behaviors.sh
```

Navigation is enabled by default. The launcher starts one Jackal, six fixed
HuNav pedestrians, Isaac WebRTC, Foxglove Bridge, `map_server`, and Arena
Nav2. The ideal D6 chassis and fixed 1/60 s physics step are also enabled by
default. Disable only Nav2 when isolating behavior work:

```bash
NAVIGATION=false GPU_ID=3 ./scripts/run_six_behaviors.sh
```

Use the previous wheel-contact skid-steer model only for an explicit comparison:

```bash
ARENA_IDEAL_CHASSIS=false GPU_ID=3 ./scripts/run_six_behaviors.sh
```

Additional launch arguments may be appended, for example:

```bash
GPU_ID=3 LIVESTREAM=false ./scripts/run_six_behaviors.sh foxglove:=false
```

### Base Arena scene

WebRTC mode:

```bash
GPU_ID=3 ./scripts/run_webrtc.sh
```

Pure headless mode without livestream:

```bash
GPU_ID=3 ./scripts/run_headless.sh
```

The base launcher uses `scene_bridge` to spawn four walls, Jackal, and one
dynamic display pedestrian. The six-behavior launcher uses its own bridge and
must not start a second `scene_bridge`, robot publisher, or Isaac process.

## Ideal D6 chassis

The workspace models the Jackal as an ideal planar velocity-controlled
industrial base by default. A D6 joint locks Z/roll/pitch while X/Y/yaw remain
free. `/cmd_vel` is bounded to 1.5 m/s and 2.0 rad/s, slew-limited to 2.0 m/s^2
and 4.0 rad/s^2, and times out after 0.5 s. Body-forward linear and yaw velocity
are applied immediately before each fixed 1/60 s PhysX step. The code never
writes pose or teleports; PhysX performs collision resolution after the actuator
update.

`/odom.twist` is populated by Isaac Sim's official
`IsaacComputeOdometry` rigid-body velocity. The HuNav bridge consumes that twist
directly rather than differentiating and filtering pose. In D6 mode its wheel
visualization commands use raw `cmd_vel` kinematics, so the old skid-steer
angular gain and static/dead-zone compensation are not applied twice.

The complete 22-case fresh-world matrix has maximum mean errors of 0.086511%
linear and 0.313368% angular. Frontal, oblique, and combined-turn wall tests all
remain collision-limited without penetration. Full tables, commands, logs,
tradeoffs, and restore paths are in `CHASSIS_CONTROL.md`.

## Six-behavior architecture

The social behavior call chain is:

```text
isaac_six_behaviors_warehouse.yaml
  -> hunav_loader
  -> hunav_agent_manager (/compute_agents)
  -> arena_humble_compat/hunav_six_behaviors_bridge
  -> arena_people_msgs/SpawnPedestrians + UpdatePedestrians
  -> arena_isaac services
  -> Isaac Sim Character pose/yaw/animation display
```

HuNav computes the pedestrian positions, velocities, orientations, and social
reactions. Isaac does not reimplement Regular, Impassive, Surprised, Scared,
Curious, or Threatening behavior.

The fixed mapping is:

| behavior.type | Name | Special configuration |
| ---: | --- | --- |
| 1 | Regular | official regular behavior, fixed cyclic lane |
| 2 | Impassive | official impassive behavior, fixed cyclic lane |
| 3 | Surprised | `once=false`, `duration=30`, `dist=4.0`, `vel=0.6` |
| 4 | Scared | `once=false`, `duration=40`, `dist=3.0`, `vel=0.6` |
| 5 | Curious | `once=false`, `duration=30`, `dist=1.5`, `vel=0.8` |
| 6 | Threatening | `once=false`, `duration=40`, `dist=1.4`, `vel=0.6` |

All six use `configuration: 1`, so `hunav_loader` does not randomly replace
their behavior parameters. Their initial positions and cyclic waypoints are
fixed in
`src/arena-rosnav/arena_bringup/configs/hunav_agents/isaac_six_behaviors_warehouse.yaml`.

The bridge schedules HuNav independently from Isaac display calls:

- simulated HuNav update target: 40 Hz
- wall scheduler: 100 Hz
- maximum integration step: 0.025 s
- Isaac Character display: 5 Hz
- measured wall-clock compute: approximately 12.6--14.7 Hz under GPU load
- measured display rate: approximately 4.9 Hz

## Navigation

The six-behavior launch now includes the same essential navigation components
as `run_common.sh`:

- `nav2_map_server/map_server` loading
  `arena_simulation_setup/worlds/map_empty/map/map.yaml`.
- `map` remapped to `/task_generator_node/map`.
- A dedicated lifecycle manager that autostarts `map_server`.
- `arena_simulation_setup/launch/nav2.launch.py` with:
  - robot `jackal`
  - global planner `navfn`
  - local planner `dwb`
  - intermediate planner `navigate_w_replanning_time`
  - `amcl=false`

Send a navigation goal with:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 5.0, y: 3.0}, orientation: {w: 1.0}}}}"
```

`bt_navigator` also directly subscribes to `/goal_pose` as
`geometry_msgs/msg/PoseStamped`, so Foxglove can publish a goal pose in the
`map` frame. The namespace-empty `/goal_pose` relay is intentionally disabled
to prevent the former self-relay loop and approximately 5000 Hz topic storm.

For manual behavior observation:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.35}, angular: {z: 0.0}}"
```

Cancel the Nav2 goal before teleoperation; do not leave two velocity publishers
controlling the robot simultaneously.

## Foxglove

The server-side bridge starts with both the base and six-behavior WebRTC
launchers. It listens on:

```text
ws://127.0.0.1:8765
```

From Windows, use an SSH tunnel unless the address is explicitly changed:

```bash
ssh -L 8765:127.0.0.1:8765 <user>@10.16.202.189
```

Then add `ws://localhost:8765` as a Foxglove connection. Useful displays are:

- 3D panel: `/tf`, `/tf_static`, `/task_generator_node/map`, `/lidar`.
- Robot model: `robot_description` and package assets.
- Plot: `/odom`, `/cmd_vel`, `/human_states`, and `/robot_states`.
- Publish panel or goal interaction: `/goal_pose` in frame `map`.

The bridge has `clientPublish`, `connectionGraph`, and `assets` capabilities.
Its allowlist exposes only selected robot mesh and common visualization asset
types.

## WebRTC

- Validated server IP: `10.16.202.189`.
- Signalling: TCP 49100.
- Media: UDP 47998 (`fixedHostPort`).
- Isaac extension: `omni.services.livestream.nvcf` 7.2.0.
- Backend: `omni.kit.livestream.webrtc` 7.0.0.

Override the detected endpoint if networking changes:

```bash
ARENA_WEBRTC_IP=<reachable-server-ip> GPU_ID=3 ./scripts/run_six_behaviors.sh
```

Both ports must be allowed end to end. TCP listening alone does not prove the
UDP media path; verify a Windows WebRTC client connection and video. The user
has already used the WebRTC view to inspect the six behavior characters, and
the latest automated run also captured a non-empty rendered frame.

## End-to-end validation

### Base Arena scene

The original WebRTC validation used:

```bash
GPU_ID=3 ARENA_SMOKE_TEST=1 ./scripts/run_webrtc.sh
```

Evidence: `logs/runs/20260823_191201_webrtc_gpu3`.

- Isaac Sim 5.1 started headless on host GPU 3.
- ROS 2 Bridge initialized.
- `map_empty` loaded as a 626 x 481 occupancy grid at 0.05 m/cell.
- Four walls, one dynamic pedestrian, and one Jackal spawned.
- Nav2 lifecycle nodes became active; `/odom` and `/lidar` were received.
- Jackal moved from `(2.999, 3.000)` to `(4.801, 3.004)`, or 1.803 m.
- WebRTC RTX encoder became ready and a rendered frame was produced.

Pure headless validation also passed in
`logs/runs/20260823_191520_headless_gpu3`; Jackal moved 1.805 m.

### Ideal D6 chassis, six behaviors, and Nav2

Final default-launch evidence:
`logs/runs/20260828_160829_six_behaviors_gpu3`.

- The launcher was invoked as `GPU_ID=3 ./scripts/run_six_behaviors.sh` with no
  D6 override and reported `Ideal D6 chassis: true`.
- Nav2 result:

  ```text
  SMOKE_NAVIGATION_OK start=(3.000,3.000) end=(4.811,3.003)
  moved=1.811m lidar_messages=84
  ```

- HuNav result:

  ```text
  SIX_BEHAVIORS_VERIFY_OK types=1,2,3,4,5,6 active=3,5
  responses=3,4,5,6 robot_distance=0.891 robot_states=87
  ```

- WebRTC produced `webrtc_frame.png` (560585 bytes), visually confirmed with
  the robot and six Characters; TCP 49100 was listening.
- Foxglove TCP 8765 was listening.
- Measured rates were about 21.2 Hz `/odom`, 3.45 Hz `/lidar`, 20.7 Hz
  `/lidar/points`, 21.4 Hz `/isaac/joint_states`, and 123 Hz aggregate `/tf`.
- `/odom` reported `child_frame_id=base_link`, official rigid-body twist near
  zero at rest, stable base Z near 0.0645 m, and effectively zero roll/pitch.
- `base_link -> lidar_link` was `[0,0,0.142]`; `base_link -> imu_link` was
  identity.
- The same final instance remained stable for approximately 5 h 46 min at
  about 14 Hz HuNav compute and 4.9 Hz display, then all children exited cleanly
  on `Ctrl-C`; ports 49100/8765 and Isaac services were released.

Dedicated chassis evidence is under `logs/chassis_control/`:

- baseline: `20260827_111238_baseline/results.csv`;
- isolated 22-case D6 matrix: `20260828_d6_isolated_full/results.csv`;
- standalone `(0.7,-0.6)`: `20260828_d6_resume_07m06_retry/results.csv`;
- three-case collision suite: `20260828_d6_collision_suite/results.csv`.

All 22 velocity rows and all three collision rows are valid. See
`CHASSIS_CONTROL.md` for the full before/after table and exact metrics.

### Earlier six-behavior deployment evidence

Pre-D6 evidence: `logs/runs/20260827_101430_six_behaviors_gpu3`.

- Six Characters spawned with behavior types `1,2,3,4,5,6`.
- `/compute_agents`, robot state, pedestrian updates, and Isaac display
  remained live during the long run.
- WebRTC produced `webrtc_frame.png` (543574 bytes).
- `map_server` reached lifecycle state `active`.
- `/task_generator_node/map` was consumed by the global costmap.
- `map -> odom` was available as an identity transform.
- `/navigate_to_pose` and the other Nav2 actions were registered.
- Navigation test result:

  ```text
  SMOKE_NAVIGATION_OK start=(2.974,3.001) end=(4.757,3.007)
  moved=1.783m lidar_messages=103
  ```

- Six-behavior regression result:

  ```text
  SIX_BEHAVIORS_VERIFY_OK types=1,2,3,4,5,6 active=3,5
  responses=3,4,5,6 robot_distance=1.032 robot_states=338
  ```

- ROS logs contain `Reached the goal!` and `Goal succeeded` for two runs.
- The final `Ctrl-C` shut down Isaac and all ROS child processes cleanly.

Re-run the checks after the scene reports `SIX_BEHAVIORS_READY`:

```bash
ros2 run arena_humble_compat verify_runtime
ros2 run arena_humble_compat verify_six_behaviors
```

## Shutdown and stale ROS cleanup

Normal shutdown is `Ctrl-C` in the launch terminal. If nodes or topics remain:

```bash
cd /home/lpc/workspace/arena5_ws
./scripts/cleanup.sh --dry-run
./scripts/cleanup.sh
```

Do not use broad `pkill`, kill other users' ROS processes, or clean an
unverified process group on this multi-user server. A topic visible after
process cleanup may be stale ROS 2 daemon data; `cleanup.sh` stops only this
workspace's configured-domain daemon.

## Local fixes and rationale

- Isaac launcher: Isaac 5.1 CLI, headless livestream, explicit signalling and
  media ports, extension enablement, and workspace-scoped logging.
- Arena messages: a local `arena_people_msgs` compatibility package for the
  Isaac pedestrian services required by the checked-out branches.
- Character display: deferred animation graph acquisition, exact commanded
  position, bounded dead reckoning, explicit stationary yaw, and correct idle
  animation.
- HuNav bridge: asynchronous compute/display scheduling, substep integration,
  fixed six-agent mapping, robot state input, and parameterized Jackal control.
- HuNav manager: zero stale velocity for Surprised and ROS-standard forward
  heading for Threatening.
- Jackal control: ideal D6 planar velocity actuator with bounded speed,
  acceleration, and timeout; legacy effective-track/angular feed-forward is
  retained only for `ARENA_IDEAL_CHASSIS=false`.
- Odometry: official `IsaacComputeOdometry` rigid-body linear/angular velocity
  is connected to `/odom.twist` and consumed directly by HuNav.
- Nav2 launch: corrected absolute map remapping and disabled the root-namespace
  self-relay for `/goal_pose`.
- Foxglove: fallback for Humble's older `ament_index_cpp` and missing direct
  `<cstdint>` include.

## Known non-blocking warnings and limitations

- Isaac's URDF importer reports unresolved intermediate `visuals/*` references
  and missing inertia for optional Jackal mount links. Articulation, wheel
  control, odometry, lidar, rendering, and navigation remain functional.
- Animation graph acquisition can warn during asynchronous Character startup;
  later logs should report `Arena dynamic pedestrian animation ready`.
- Launch prints `Failed to load entry point 'NodeLogLevelExtension': No module
  named 'arena_bringup.extensions'`. This is noisy but non-fatal.
- Time-based Nav2 replanning can abort/replace an internal `follow_path` handle
  each second. Judge the top-level navigation result, controller progress, and
  final `Goal succeeded`, not those replacement warnings alone.
- Scared can be visually subtle with the official parameter set. Tune the demo
  YAML rather than implementing reaction logic in Isaac.
- HuNav social forces are not a strict collision-proof constraint. Scene
  geometry and update timing should be retested after behavior tuning.
- D6 is intentionally an ideal actuator rather than a tyre model. It refreshes
  root twist before every PhysX step; structural changes must be checked with
  the complete fresh-world matrix and frontal/oblique/combined wall suite.
- The current Jackal import reports `disabled_wheel_colliders=0`. This exact
  import state passed the full matrix and collision suite; wheel-collider
  traversal is therefore optional, not a hidden validation requirement.
- `behavior.state` is not a sufficient verifier for Scared and Threatening
  because their upstream action path can reset it during `computeForces()`;
  the supplied verifier also checks kinematic outcomes.

## Patch records

Current records in `logs/upstream_diffs/`:

```text
arena-isaac.diff
arena-rosnav.diff
foxglove-sdk.diff
robot-angular-velocity-fix.diff
simulation-setup.diff
six-behaviors-arena-isaac.diff
six-behaviors-arena-rosnav.diff
six-behaviors-hunav-sim.diff
```

The source trees are authoritative. Some named six-behavior patch files were
captured before navigation was added to the launch; regenerate them before
using this directory as a complete archival patch set.

## Directory layout

```text
arena5_ws/
├── .cache/                  # Arena-scoped XDG, pip and Conda caches
├── .conda/arena_ros/        # workspace-local ROS 2 Humble environment
├── HANDOFF.md               # current state, blockers, next plan and pitfalls
├── DEPLOYMENT.md            # deployment and operating guide
├── CHASSIS_CONTROL.md       # ideal D6 design, matrix/collision evidence, restore
├── build/                   # colcon build products
├── config/generated/        # generated Jackal URDF
├── install/                 # colcon install space and staged lightsfm headers
├── log/                     # colcon logs
├── logs/
│   ├── runs/                # per-run Isaac, ROS, Nav2 and frame evidence
│   └── upstream_diffs/      # local upstream patch records
├── scripts/
│   ├── env.sh
│   ├── build.sh
│   ├── cleanup.sh
│   ├── ideal_chassis_matrix.py
│   ├── ideal_chassis_collision.py
│   ├── run_common.sh
│   ├── run_foxglove.sh
│   ├── run_headless.sh
│   ├── run_ideal_chassis_test.sh
│   ├── run_ideal_chassis_matrix_isolated.sh
│   ├── run_ideal_chassis_collision_suite.sh
│   ├── run_six_behaviors.sh
│   └── run_webrtc.sh
└── src/
    ├── arena-rosnav/
    ├── arena-isaac/
    ├── arena/               # simulation-setup, evaluation and tools
    └── deps/                # pinned Nav2, HuNav, Jackal, Foxglove, lightsfm
```

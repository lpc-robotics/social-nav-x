# Arena 5 / Isaac Sim 5.1 handoff

Updated: 2026-08-28 21:55 CST. Workspace:
`/home/lpc/workspace/arena5_ws`.

## Current task

The latest task restored and completed the archived ideal-D6 chassis candidate.
That task is complete: the D6 controller is now the default in both normal and
six-behavior launchers, and the old PhysX skid-steer model remains available as
an explicit fallback. See `CHASSIS_CONTROL.md` for the design, full before/after
matrix, collision results, evidence paths, and recovery procedure.

The six-behavior launcher starts, by default:

- Isaac Sim 5.1 headless with WebRTC.
- The Isaac ROS 2 Bridge.
- One Jackal and six Isaac Characters driven by HuNavSim.
- Foxglove Bridge.
- `map_server` for `map_empty`, remapped to
  `/task_generator_node/map`, and its lifecycle manager.
- Arena Nav2 using NavFn, DWB, `navigate_w_replanning_time`, and
  `amcl=false`.
- The ideal planar chassis at fixed 1/60 s physics, with PhysX collision still
  active and official rigid-body velocity published in `/odom.twist`.

The final validation instance ran for approximately 5 h 46 min, then stopped
cleanly with `Ctrl-C`. At handoff there are no Arena/Isaac processes, Isaac ROS
services, or listeners on TCP 49100/8765. Use the scoped cleanup procedure
below after an abnormal exit.

## Start here

```bash
cd /home/lpc/workspace/arena5_ws
source scripts/env.sh
GPU_ID=3 ./scripts/run_six_behaviors.sh
```

Ideal D6 is enabled by default. To compare against the previous skid-steer
fallback without changing source:

```bash
ARENA_IDEAL_CHASSIS=false GPU_ID=3 ./scripts/run_six_behaviors.sh
```

Navigation is enabled by default. To run only the six behaviors:

```bash
NAVIGATION=false GPU_ID=3 ./scripts/run_six_behaviors.sh
```

Before restarting after an abnormal exit:

```bash
./scripts/cleanup.sh --dry-run
./scripts/cleanup.sh
```

The cleanup script is scoped to the current UID, this workspace path, and
the configured ROS domain. It sends `SIGINT`, then `SIGTERM`, then `SIGKILL`
only when needed, and stops the workspace ROS 2 daemon to remove a stale CLI
graph.

## Environment snapshot

- Ubuntu 22.04.5 LTS, kernel 6.8.0-136-generic.
- Four NVIDIA GeForce RTX 4090 GPUs, 24564 MiB each.
- NVIDIA driver 580.126.09.
- ROS 2 Humble in `.conda/arena_ros`; Python 3.11.15.
- Isaac Sim 5.1.0.0 reused from
  `/home/lpc/miniforge3/envs/isaaclab`.
- Arena-Rosnav branch `humble`, commit
  `c2ff4a87e8686013b53f1e9cd8b01b3ab04fbce4` plus local changes.
- Arena Isaac branch `arena5-isaac5.1.0`, commit
  `16b8e3416517d8c3dc1b5038df4fe11b9a6df46c` plus local changes.
- HuNavSim v1 commit
  `a69cf96d98b0d40e247f819d7aebab661ac68b3b` plus a small behavior fix.
- Foxglove Bridge 3.4.3, built from Foxglove SDK commit
  `05f27efc7e535d9c30c6b0cb4f6aa89de7243870` plus Humble compatibility fixes.
- `ROS_DOMAIN_ID=51`, `rmw_fastrtps_cpp`, ROS CLI daemon disabled by default.
- Current filesystem free space: approximately 265 GiB.
- At handoff, GPU 3 is free except for about 36 MiB of baseline allocation.
- No `sudo`, `apt`, `apt-get`, Docker, or `pip install --user` is required.

`GPU_ID=3` sets `CUDA_VISIBLE_DEVICES=3`; code inside the isolated process must
use CUDA device `cuda:0`. Kit/Vulkan still receives host render ordinal 3.

## Completed work

### Base deployment and visualization

- Reused the existing ROS, CUDA driver, Conda, and Isaac Sim installations.
- Kept build, install, caches, generated URDF, logs, and source dependencies
  under the single Arena workspace.
- Added Isaac 5.1 headless/WebRTC options. The validated endpoint is
  `10.16.202.189`, signalling TCP 49100 and fixed media UDP 47998.
- Built Foxglove Bridge 3.4.3 locally with ROS 2 Humble API compatibility.
  It listens on `127.0.0.1:8765` by default and supports client publishing,
  connection graph inspection, and package/file assets.
- Added `scripts/env.sh`, `build.sh`, `run_headless.sh`, `run_webrtc.sh`,
  `run_foxglove.sh`, and workspace-scoped `cleanup.sh`.

### Arena / Isaac compatibility chain

The live six-behavior chain is:

```text
HuNav YAML -> hunav_loader -> hunav_agent_manager /compute_agents
           -> arena_humble_compat/hunav_six_behaviors_bridge
           -> arena_people_msgs SpawnPedestrians / UpdatePedestrians
           -> arena_isaac services -> Isaac Character display
```

The bridge also spawns the arena walls and Jackal, publishes wheel commands,
and supplies robot pose plus official Isaac rigid-body velocity to HuNav from
`/odom`. HuNav owns all social behavior logic; `Person.py` only renders
commanded pose, yaw, velocity, and animation.

### Six fixed HuNav behaviors

Added
`arena_bringup/configs/hunav_agents/isaac_six_behaviors_warehouse.yaml` and
`arena_bringup/launch/isaac_six_behaviors.launch.py`.

The fixed mapping is:

| ID/type | Name | Current role |
| --- | --- | --- |
| 1 | Regular | follows its cyclic lane normally |
| 2 | Impassive | follows its lane without special robot reaction |
| 3 | Surprised | stops and turns to watch the robot |
| 4 | Scared | moves away when the robot is visible |
| 5 | Curious | approaches to its configured robot distance |
| 6 | Threatening | seeks a point in front of the robot |

The four special behaviors use `once: false` so that they can be triggered
repeatedly during an interactive observation. Behavior parameters are based
on the HuNav v1 warehouse demo; scene positions and lanes were made fixed and
separated to avoid an initial path crossing through the robot.

### HuNav timing and Character display fixes

- Decoupled HuNav computation from the slower Isaac update service.
- Configured a 40 Hz simulated compute target, a 100 Hz scheduler, a 5 Hz
  Isaac display rate, and a maximum integration substep of 0.025 s.
- Observed approximately 12.6--14.7 Hz wall-clock HuNav compute and 4.9 Hz
  Isaac display in a long GPU-3 run; this remains within the requested
  10--20 Hz compute range.
- Corrected the HuNav Threatening forward point from swapped
  `(sin(yaw), cos(yaw))` to ROS-standard `(cos(yaw), sin(yaw))`.
- Cleared Surprised's stale velocity when it stops, preventing a stationary
  character from playing a walking state.
- Passed HuNav orientation through `UpdatePedestrians` and applied it at zero
  speed in the Character bridge, so Surprised turns in place without adding
  behavior logic to Isaac.
- Added timestamp-aware Character dead reckoning between the 5 Hz display
  updates without blocking HuNav integration.

### Ideal D6 chassis control

The former wheel-contact skid-steer path produced severe, operating-point
dependent angular attenuation, especially during combined linear/angular
motion. The current controller instead models an industrial velocity-controlled
planar base:

- world D6 constraint locks Z/roll/pitch and leaves X/Y/yaw free;
- bounded `/cmd_vel` references use 1.5 m/s and 2.0 rad/s limits;
- 2.0 m/s^2 and 4.0 rad/s^2 slew limits plus a 0.5 s timeout;
- body-forward linear and yaw velocity are applied before each 1/60 s PhysX
  step; pose is never written;
- PhysX still resolves chassis contacts and collisions after the velocity
  actuator is applied;
- `/odom.twist` comes from `IsaacComputeOdometry`, not pose differentiation;
- the old separation multiplier and angular gain/static term are bypassed in
  D6 mode, preventing duplicate compensation.

All 22 isolated matrix/generalization rows are valid. Maximum mean error was
0.086511% linear and 0.313368% angular, well below the requested 3%/5% bounds.
Frontal, oblique, and combined-turn wall tests all stopped without penetration.
Exact implementation and tradeoffs are documented in `CHASSIS_CONTROL.md`.

### `/goal_pose` feedback loop

The earlier approximately 5000 Hz `/goal_pose` storm came from a relay whose
input and output resolved to the same absolute topic when the robot namespace
was empty. `simulation-setup/launch/nav2.launch.py` now starts that relay only
for a non-empty namespace. With the current root namespace, `bt_navigator`
subscribes directly to `/goal_pose` and no relay is launched.

### Navigation merged into the six-behavior launcher

`isaac_six_behaviors.launch.py` now conditionally starts the same map and Nav2
configuration as `run_common.sh`. `run_six_behaviors.sh` exports and validates
`NAVIGATION`, checks its built packages, and passes the option into the launch.

The final default-launch validation on 2026-08-28 produced:

```text
SMOKE_NAVIGATION_OK start=(3.000,3.000) end=(4.811,3.003)
moved=1.811m lidar_messages=84

SIX_BEHAVIORS_VERIFY_OK types=1,2,3,4,5,6 active=3,5
responses=3,4,5,6 robot_distance=0.891 robot_states=87
```

The final evidence directory is
`logs/runs/20260828_160829_six_behaviors_gpu3`. It contains a 560585-byte
WebRTC frame, ROS logs, a Nav2 goal success, the six-behavior marker, and
odom/TF/sensor evidence. It continued running for approximately 5 h 46 min at
about 14 Hz HuNav compute and 4.9 Hz display before a clean shutdown. A separate
approximately ten-minute run is
`logs/runs/20260828_155530_six_behaviors_gpu3`.

## Build and verification commands

Full workspace rebuild:

```bash
cd /home/lpc/workspace/arena5_ws
source scripts/env.sh
./scripts/build.sh
```

The final chassis/odom/HuNav bridge changes were also rebuilt successfully with:

```bash
colcon build --packages-select arena_isaac arena_humble_compat
```

Runtime verification, after the scene reports `SIX_BEHAVIORS_READY`:

```bash
ros2 run arena_humble_compat verify_runtime
ros2 run arena_humble_compat verify_six_behaviors
```

Send a Nav2 goal:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 5.0, y: 3.0}, orientation: {w: 1.0}}}}"
```

Manual robot control for behavior observation:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.35}, angular: {z: 0.0}}"
```

Do not publish teleoperation commands while a Nav2 goal is active: both paths
write robot velocity and will fight each other.

## Pitfalls encountered

- A large ROS topic list after stopping a launcher is not proof that Isaac is
  still running. Check this UID's process groups first, then stop the
  workspace's ROS 2 daemon; cached graph data and orphaned processes require
  different fixes.
- A relay from `/goal_pose` to `goal_pose` becomes a self-loop in the root
  namespace. It produced the observed multi-kilohertz feedback storm. Resolve
  launch substitutions to absolute names before adding relays.
- Starting Nav2 nodes or compiling successfully is not an end-to-end test.
  Confirm lifecycle state, map subscription, TF, action registration, odometry,
  lidar, physical motion, and the top-level action result.
- Calling the blocking Isaac `UpdatePedestrians` service in the HuNav compute
  callback reduced social integration to the display rate and made large `dt`
  jumps look like collisions. Compute and display must remain asynchronous,
  with bounded integration substeps.
- Isaac's animation graph owns the rendered Character root after attachment.
  Writing only USD transforms can update internal state while leaving the
  visible model stuck. Pose/yaw updates must go through the Character graph.
- A zero-speed Surprised agent can still need a yaw update. Inferring heading
  only from velocity prevents the in-place turn; orientation must be carried
  separately while keeping HuNav as the behavior authority.
- ROS yaw is +X-forward: `(cos(yaw), sin(yaw))`. The upstream Threatening code
  used swapped sine/cosine components and targeted the wrong side of the robot.
- With `CUDA_VISIBLE_DEVICES=3`, Isaac-side CUDA device 3 is invalid; the
  selected physical GPU is exposed internally as device 0.
- The D6 path is an ideal velocity actuator, not a detailed tyre model. It
  refreshes root twist before each physics step and deliberately prioritizes
  `cmd_vel -> chassis twist` over skid-steer slip realism. Collision is still
  post-solved by PhysX; rerun the supplied wall suite after changing the
  constraint, step ordering, collision geometry, or command limits.
- The current Jackal import logs `disabled_wheel_colliders=0` because its wheel
  collider prims are not exposed below the traversed robot hierarchy. The 22
  velocity cases and three collision cases passed in this exact state, so do
  not treat that counter as an unverified prerequisite or tune around it.
- Foxglove Bridge 3.4.3 assumes a newer `ament_index_cpp/version.h`; Humble needs
  the included fallback. Installing a second ROS distribution is unnecessary.
- ROS 2 Humble CLI compatibility differs from newer examples:
  `ros2 action list` does not accept `--no-daemon`, and this `tf2_echo` does not
  offer `--once`. Use plain action listing and a bounded `timeout` for TF.
- The workspace root is not a Git repository. Run status/diff commands against
  each nested repository and do not assume one top-level commit captures all
  deployment changes.

## Current blockers and open issues

There is no hard deployment or chassis blocker. The current code builds; the
complete D6 velocity/collision suite and navigation plus six-behavior regression
pass. The remaining items are non-blocking engineering concerns:

1. The source trees contain intentional, uncommitted modifications and new
   compatibility packages. Do not run `git reset`, checkout over them, or pull
   an upstream branch without first saving the diffs.
2. Scared is less visually dramatic than some other behaviors under the
   official warehouse parameters. Its `dist`, `vel`, and starting geometry can
   be tuned in the demo YAML, but that is a scenario-tuning decision, not an
   Isaac rendering defect.
3. HuNav's social-force model reduces collisions but does not provide a hard
   geometric non-collision guarantee, especially while special behaviors
   dynamically replace goals. Retest after any waypoint, `dist`, velocity, or
   frequency change.
4. Nav2's time-based replanning tree can print repeated `follow_path` action
   abort/replacement warnings as it hands a new path to DWB. In the validated
   run the controller still logged `Reached the goal!` and BT Navigator logged
   `Goal succeeded`; those replacement warnings alone are not a failure.
5. Launch emits `Failed to load entry point 'NodeLogLevelExtension': No module
   named 'arena_bringup.extensions'`. This is noisy but did not prevent any
   node from starting or navigating.
6. `ScaredNav` and `ThreateningNav` call `computeForces()`, which can reset
   `behavior.state` before serialization. The verifier therefore checks their
   kinematic response, not only the state bit.
7. WebRTC still depends on TCP 49100 and UDP 47998 being reachable from the
   Windows client. Foxglove defaults to loopback and should normally be
   reached through an SSH tunnel to TCP 8765.
8. `verify_six_behaviors` observes transient HuNav states. Curious can enter its
   state before a post-navigation verifier subscribes; repeat the verifier in
   the same unmodified run if only type 5 is initially missing. The final run
   did so and produced the required `types=1,2,3,4,5,6` marker.

## Next plan

1. Keep D6 as the default unless a structural failure is reproduced. Do not
   return to operating-point gain/static compensation for routine accuracy
   work.
2. After any chassis, odom, constraint, or collision change, rerun the complete
   fresh-world matrix and all three wall cases from `CHASSIS_CONTROL.md`; a
   single `(v,w)` probe is not sufficient.
3. Run Nav2 before the motion-producing six-behavior verifier in a fresh system
   regression so each verifier's displacement threshold has a known start.
4. If tuning Scared, change only the independent demo YAML first. Compare one
   parameter at a time and leave Isaac Character and chassis logic untouched.
5. Refresh the patch records and create commits or an external archive before
   any upstream sync. The workspace root itself is not a Git repository; the
   Arena, Isaac, simulation-setup, HuNav, and Foxglove trees are independent.

## Patch and source-change inventory

Current patch records are under `logs/upstream_diffs/`:

- `arena-isaac.diff`: initial Isaac 5.1/Humble compatibility work.
- `simulation-setup.diff`: Nav2 map remapping and `/goal_pose` relay fix.
- `foxglove-sdk.diff`: ROS 2 Humble compile compatibility.
- `six-behaviors-arena-isaac.diff`: six-behavior bridge and Character updates.
- `six-behaviors-arena-rosnav.diff`: six-behavior YAML/launch/package changes.
- `six-behaviors-hunav-sim.diff`: Surprised stopped state and Threatening yaw.
- `robot-angular-velocity-fix.diff`: Jackal angular feed-forward calibration.

The feed-forward patch is retained as history/fallback evidence; it is not the
active D6 control path. The pre-D6 baseline and finalized D6 snapshots are under
`backups/`; exact paths and restore commands are in `CHASSIS_CONTROL.md`.

Some records were created before the latest navigation addition to the
six-behavior launch. Regenerate them from the current working trees before
treating the directory as a complete archival snapshot.

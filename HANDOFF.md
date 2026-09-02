# Arena 5 / Isaac Sim 5.1 handoff

Original deployment handoff updated: 2026-08-28 21:55 CST. Formal-social
delivery and activity-source integration addendum updated: 2026-09-02. Workspace:
`/home/lpc/workspace/arena5_ws`.

## Activity source integration (2026-09-02)

The formal-social sources and the validated Isaac Character frame correction
are now present in the activity source tree. Formal-package builds remain
isolated; the later authorized shared `arena_isaac` deployment is recorded
below:

```bash
cd /home/lpc/workspace/arena5_ws
env ARENA_BASE_WS="$PWD" FORMAL_OVERLAY_ROOT="$PWD/.colcon-formal-v1" \
  scripts/build_formal_overlay.sh
env ARENA_BASE_WS="$PWD" FORMAL_OVERLAY_ROOT="$PWD/.colcon-formal-v1" \
  scripts/test_formal_overlay.sh
```

Do not run a full shared build and do not install or update dependencies. The
merged overlay passed Character frame `17/17`, compat
`20/20`, formal `76/76`, with xUnit `20/261` and no errors, failures or skips.

The activity copy did have the same approximate 90-degree visual-heading bug.
Before merge, its actual `Person.py` SHA-256 was
`883c1aaec2c242521015313c57f8ce562d5a6122ada27b6c4bcb6a58615df684`
and `character_frames.py` did not exist. ROS yaw uses local `+X` as forward,
while the Isaac People asset uses local `-Y`. The activity source now applies
`q_character = q_ros * qz(+pi/2)` and the inverse
`q_ros = q_character * qz(-pi/2)` only at the Character boundary. This keeps
HuNav and `/human_states` in ROS semantics.

An activity sudden run passed with zero stop speed and `2.883 degrees` facing
error; the original strict entry still produced
`SIX_BEHAVIORS_VERIFY_OK types=1,2,3,4,5,6`. Detailed hashes, logs and the exact
allowlist are in
`docs/formal_social_automata/ACTIVITY_SOURCE_MERGE_20260902.md`.

### Shared arena_isaac deployment (authoritative latest state, 2026-09-02)

At the user's direction, the already merged activity source was selectively
built into the shared `build/arena_isaac` and `install/arena_isaac`:

```bash
cd /home/lpc/workspace/arena5_ws
source scripts/env.sh
colcon build --event-handlers console_cohesion+ \
  --packages-select arena_isaac \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Only `arena_isaac` was built; the result was `1 package finished`. Against the
pre-deployment install snapshot, the runtime source delta is exactly the
replacement of `pedestrian/.../person.py`, addition of
`pedestrian/.../character_frames.py`, and generated metadata/bytecode. Both
installed source hashes match activity source:

```text
person.py            429a25528bb9cc5cd7b16616a6099d0644777e90155e501ff0099c544adfa7f1
character_frames.py  2e018db9ad9c34c8e4cedb057637628d08f6c7dd4be0fe74fe5ac5cf47d01eaa
```

Colcon also refreshed the timestamps of its standard top-level generated
`install/setup*` and `local_setup*` files. The package inventory did not change,
and `install/setup.bash` retained SHA-256
`e3b0addf5e333f92b50598538d132869b8ee08bcad6cc4ef3e9361fdfaada898`.

The main launcher now defaults to the validated shared install. It rejects a
stale package before starting Isaac; the fast check is:

```bash
GPU_ID=3 ./scripts/run_six_behaviors.sh --check-runtime-only
```

The launcher commit is `d8b026d`. Set
`ARENA_SIX_BEHAVIORS_USE_OVERLAY=true` only for an explicit comparison with
`.colcon-formal-v1`. The shared-path Character tests passed `17/17`; a GPU 3 run
confirmed the actual executable
`/home/lpc/workspace/arena5_ws/install/arena_isaac/lib/arena_isaac/run_isaacsim`
and returned `SIX_BEHAVIORS_VERIFY_OK types=1,2,3,4,5,6 active=3,5
responses=3,4,5,6 robot_distance=0.967 robot_states=372`. Runtime samples were
`14.6--15.1 Hz` compute, `4.8 Hz` display and `max_dt=0.025 s`.

Evidence is at
`logs/regression/shared_install_character_frame_fix_20260902/verification.txt`
(SHA-256
`72ef2199f4cc5fa56f9d5c84ab7d35e58c97a6a44adc206c098ef30dc2b39596`)
and `logs/runs/20260902_170015_six_behaviors_gpu3/`. Restore only by following
`/home/lpc/workspace/arena5_ws_archives/20260902_shared_arena_isaac_install_pre/RESTORE.md`;
the pre-deployment archive SHA-256 is
`6905ce412deb772da3c04819f6942558707b0bcb98a21996c9cddb8070466f4d`.
Conda, dependencies, other shared packages and nested Git indexes were not
changed.

### Intermediate overlay workaround (historical, superseded above)

The source merge was correct, but the activity `run_six_behaviors.sh` still
sourced only the legacy shared `install`. That installed `Person.py` has SHA-256
`883c1aaec2c242521015313c57f8ce562d5a6122ada27b6c4bcb6a58615df684`,
so the main entry could still render the old 90-degree error even though the
source and `.colcon-formal-v1` overlay were fixed.

At this intermediate checkpoint, the launcher loaded `.colcon-formal-v1` by
default, checked that both
`arena_isaac` and `arena_humble_compat` resolve there, byte-compares the
installed `Person.py` with activity source, and fails before Isaac starts if the
overlay is absent or stale. The implementation commit is `0359579`. No manual
`source` is needed:

```bash
cd /home/lpc/workspace/arena5_ws
GPU_ID=3 ./scripts/run_six_behaviors.sh

# Fast non-launching preflight
GPU_ID=3 ./scripts/run_six_behaviors.sh --check-overlay-only
```

The verified Isaac executable was
`.colcon-formal-v1/install/arena_isaac/lib/arena_isaac/run_isaacsim`; the main
run returned `SIX_BEHAVIORS_VERIFY_OK types=1,2,3,4,5,6 active=3,5
responses=3,4,5,6 robot_distance=1.020 robot_states=456`. Character-frame tests
remain `17/17`, steady compute was `14.848--18.399 Hz`, display
`4.806--4.891 Hz`, and `max_dt=0.025 s`.

The conversion is shared by Regular, Impassive, Surprised, Scared, Curious and
Threatening. It corrects only the rendered body axis and inverse ROS feedback;
it does not change HuNav goals, velocity, forces, behavior types or transition
logic. Surprised now renders its stationary look-at yaw correctly; moving
profiles render their body consistently with the ROS trajectory.

Evidence is under
`/home/lpc/workspace/arena5_ws/logs/regression/six_behavior_overlay_heading_fix_20260902/`
and `/home/lpc/workspace/arena5_ws/logs/runs/20260902_163617_six_behaviors_gpu3/`.
The targeted pre-change launcher backup is
`/home/lpc/workspace/arena5_ws_archives/20260902_six_behavior_overlay_entry_pre/`;
its `RESTORE.md` SHA-256 is
`2461d12027ee7b8491417fd34af403368cf7a162e1272f41d04ec7d387e9bc99`.
At that checkpoint the shared install remained unchanged. Setting
`ARENA_SIX_BEHAVIORS_USE_OVERLAY=false` selects that legacy install only for
diagnosis/rollback and can reproduce the old visual error.

The workspace root is not Git. Run manifests use merged-content revision
`3ac7a64e30f381221cc0835059ac61c43e86961f603e582625bf58d529e7e2f4`.
At the initial merge/overlay checkpoints the shared install, Conda environment
and nested Git indexes remained unchanged. The latest authorized deployment
changes only shared `build/arena_isaac` and `install/arena_isaac`; Conda and
nested Git indexes remain unchanged.
The six-behavior launcher is now the one intentional follow-up change described
in the historical overlay checkpoint; its SHA-256 at that checkpoint was
`24abb1ee73e2ca66aad1c352570c2e1d757fcfadb7ee9f1617c768f54d7f9633`.

Rollback instructions are at
`/home/lpc/workspace/arena5_ws_archives/20260902_formal_source_merge_pre/RESTORE.md`;
the protected archive SHA-256 is
`024672330865d7500ee2af9045e0d976bb9dc5f3f8f398ddb96dcb24b9fa6e27`.
Do not overlay-extract it and do not reset or clean nested repositories.

At the user's explicit direction, chassis and collision follow-up testing was
stopped. The retry has `18/22` valid chassis results; the remaining `4/22` and
all collision cases (`0/3` run) are **not executed**, not passed. No test or
Isaac process remains running.

## Formal social automata V1 delivery (2026-09-01)

This section records the original isolated formal-social delivery. The section
above is authoritative for its later activity-source integration; the rest of
this file remains deployment/D6 history.

- Feature worktree: `/home/lpc/workspace/social-nav-x-formal-v1`
- Branch: `feature/formal-social-automata-v1`
- Canonical base: `51ab117dedf6a8173c1704f0edd8d01c7938fb8e`
- Base tag: `arena5-isaac5.1-archive-20260829`
- Implementation commits: `ec95e8c`, `eaa84c7`, `e33dd6d`, `a048bfd`,
  `0359579`, `d8b026d`; the
  later Regular-motion/visual and Character-frame fixes are identified by the
  final feature `HEAD` (`git rev-parse HEAD`).
- Authoritative specification:
  `FORMAL_SOCIAL_AUTOMATA_DEVELOPMENT_PLAN.md`
- Immutable source request:
  `docs/formal_social_automata/source_spec_20260830.md`, SHA-256
  `e033334817f3a6096845765969c5af57444644d32d6df618b34de16b9f12a268`.

The delivered V1 is a deterministic `1 Robot + 1 Human` pipeline with
`NORMAL/ATTENTION/CURIOUS/SURPRISED/SCARED`, Schmitt-trigger events, simulation
time dwell/recovery/cooldown, full HuNav profiles, transactional reset/compute,
stable JSON topics/JSONL traces, a one-human launch, and a repeatable GPU
acceptance driver. `THREATENING` remains only in the original six-behavior demo;
multi-human/shared events, `SOCIAL`, RL, probability, YAML guard DSL and UPPAAL
remain out of scope.

Build and test only an isolated overlay. In the feature worktree:

```bash
cd /home/lpc/workspace/social-nav-x-formal-v1
scripts/build_formal_overlay.sh
scripts/test_formal_overlay.sh
```

The final test summary is:

```text
character frame conversion: 17 passed
arena_humble_compat: 20 passed
formal_social_behavior: 76 passed
colcon compat: 20 tests, 0 errors, 0 failures, 0 skipped
colcon formal: 261 tests, 0 errors, 0 failures, 0 skipped
```

The three pytest runs collect 113 top-level cases; colcon/xUnit expands the
formal unittest subtests to 261. Raw colcon logs are under
`.colcon/test-log/test_2026-09-01_21-12-30/`; the only warnings are two
dependency-side Lark deprecations. The build selects `arena_isaac`,
`arena_humble_compat` and `formal_social_behavior`; `arena_isaac` resolves to
the feature overlay rather than the shared activity install. The activity-source
layout uses `.colcon-formal-v1` as described above.

Run one formal demo (the script sources the activity underlay and feature
overlay itself):

```bash
cd /home/lpc/workspace/social-nav-x-formal-v1
env DRL_VO_GUI=false GPU_ID=3 NAVIGATION=false \
  scripts/run_formal_social_demo.sh \
  headless:=true livestream:=false foxglove:=false
```

For human-visible validation, keep the default WebRTC launch running in the
first terminal and run exactly one visual driver in a second terminal:

```bash
# terminal 1 (headless rendering with WebRTC enabled by default)
GPU_ID=3 NAVIGATION=false scripts/run_formal_social_demo.sh

# terminal 2: safe defaults to 8 s; sudden/fast default to 2 s
scripts/run_formal_social_visual_scenario.sh safe
scripts/run_formal_social_visual_scenario.sh sudden 2.0
scripts/run_formal_social_visual_scenario.sh fast 2.0
```

The visual driver prints each formal state/transition plus half-second samples
of behavior type, distance, speed, ROS-logical quaternion yaw, robot-target yaw
and facing error. The Character boundary applies the asset-axis conversion
described below. It writes `visual.log` and `run_manifest.txt` under
`logs/formal_visual/`; it does not launch a second Isaac instance or freeze the
automaton. Run only one scenario driver at a time so it remains the sole
`/cmd_vel` publisher.

Run the required two-round GPU matrix:

```bash
env DRL_VO_GUI=false GPU_ID=3 FORMAL_ACCEPTANCE_DOMAIN_BASE=181 \
  scripts/test_formal_simulation_matrix.sh
```

The authoritative matrix is:

```text
/home/lpc/workspace/social-nav-x-formal-v1/logs/formal_acceptance/20260901_173536_764353999_pid127944
FORMAL_SOCIAL_ACCEPTANCE_MATRIX_OK cases=6 rounds=2 scenarios=safe,sudden,fast
```

Its `matrix.log` SHA-256 is
`169b55b0c739a36227b7b761314655984262092eaeff0b0e53cd97424db0aecb`.
Safe reproduced `ATTENTION->CURIOUS->NORMAL` twice; sudden reproduced
`ATTENTION->SURPRISED->NORMAL` twice; fast reproduced
`ATTENTION->SCARED->NORMAL` twice. Across 12 distinct pre/post-action intervals,
all six one-goal recoveries reported `regular_motion=stopped`. Steady HuNav
compute was `10.882--22.375 Hz`, Character display was
`4.678--5.929 Hz`, maximum integration step was `0.025 s`, and lag was
`0.005--0.008 s`.

The fixed failure was not an automaton transition error. With the single cyclic
goal at `(6,3)`, HuNav's Regular tree considers `goal_radius + 0.1 m` reached,
then only rotates that same goal and does not tick RegularNav or clear the
velocity left by Curious. Isaac was repeatedly re-anchored to the fixed pose
while receiving a non-zero velocity, so its walk animation played in place.
The proxy now zeroes all linear/angular motion only on copied Regular messages
inside that exact reached-goal boundary, both before reset/compute and on the
first raw response entering it. Pose, yaw, goal, caller request and HuNav source
are unchanged. This sentence describes the original feature-only validation;
the activity Character source was later intentionally synchronized under the
protected 2026-09-02 merge.

Isaac People characters use local `-Y` as their visual forward axis, whereas
ROS planar poses use local `+X`. The feature `Person.py` now converts only at
the Character Graph boundary:
`q_character = q_ros * qz(+pi/2)`, with the inverse
`q_ros = q_character * qz(-pi/2)` on feedback. `/human_states` therefore
retains ROS semantics, while the rendered native `-Y` front points along the
same world heading. The pure conversion tests pass 17 cases over wraparound,
round trip and invalid input.

Production-style visual retests on GPU 3 passed all modes. Sudden stopped at
zero speed and `2.862 degrees` robot-facing error; safe Curious reduced distance
from `2.480 m` to `2.333 m`; fast Scared increased distance by `1.146181 m`
with exit closing speed `-0.839761 m/s`. The relevant raw logs are:

```text
logs/formal_visual/20260901_194912_743233044_sudden_pid548382/visual.log  a10f4a3a60f58a438b75416cc455e40ed6e91cc63bf57ee738d9842585b6a694
logs/formal_visual/20260901_195144_636426696_safe_pid557741/visual.log    45f211e5ffa60707d7994c0adc8e6cad568542025e6be6017becf4215738e13f
logs/formal_visual/20260901_210908_750275095_fast_pid795427/visual.log    2bd9cb5bc96c1df699e22370eb719ee5f392fe07eaa3a93ff86dd38d19ed13a1
```

Effective configuration hashes:

```text
formal_social_automata.yaml  5a8be544cabfc0d7f6546c496d22f4ec521fc366d578578f36267b4d53857693
formal_social_agent.yaml     a3f03ea3ea74178377e06aea86e764f47225e3daeb90abd219080257f78b47b6
```

The final matrix deliberately gives every case its own ROS domain. The fast
case uses odometry-gated `0.8 m/s` pulses with test-process-only
`physics_dt=0.01 s`, `100 m/s^2` acceleration and `0.012 s` command watchdog;
production defaults stay `1/60 s`, `2.0 m/s^2` and `0.5 s`. The verifier
requires type 4, a measured distance increase, negative closing speed and
positive human-outward speed reconstructed from matching odom/state stamps if
the display frame is missed. The two runs increased distance by
`1.064276/0.000962 m`, with exit closing speeds
`-0.799999/-0.038465 m/s` and human-outward speeds
`0.799999/0.799999 m/s`. After the frame fix exposed a real FOV loss when
Scared turns away, `ROBOT_LOST` recovers Scared only when no danger event is
active. A danger+lost sample stays Scared and clears safe timing, preventing a
Regular/Scared reset storm; a later safe lost still recovers immediately.

Regression evidence:

```text
logs/regression/original_no_overlay_20260831_unsandboxed/
  SMOKE_NAVIGATION_OK ... moved=1.885m lidar_messages=96
  SIX_BEHAVIORS_VERIFY_OK types=1,2,3,4,5,6
logs/regression/original_with_overlay_final_20260831/
  overlay prefix: /home/lpc/workspace/social-nav-x-formal-v1/.colcon/install/arena_humble_compat
  SIX_BEHAVIORS_VERIFY_OK types=1,2,3,4,5,6 active=3,5 responses=3,4,5,6 robot_distance=0.952 robot_states=341
logs/regression/original_with_character_frame_fix_20260901/
  arena_isaac + compat prefixes: /home/lpc/workspace/social-nav-x-formal-v1/.colcon/install/...
  SIX_BEHAVIORS_VERIFY_OK types=1,2,3,4,5,6 active=3,5 responses=3,4,5,6 robot_distance=1.035 robot_states=302
```

`patches/arena-isaac.patch` has SHA-256
`1d404fca247c81a6dfbfaa470cfd35a7ca8005d0b2cd2be056fd9bf141875b23`.
Forward apply, applied-tree reverse and all 31 file comparisons pass against
fixed upstream `16b8e3416517d8c3dc1b5038df4fe11b9a6df46c`; verification tree:
`/tmp/social-nav-x-arena-isaac-verify-character-frame.7Xjf7m`. No merge to
`main` and no remote push were performed.

Runtime deactivation is immediate: stop the formal launch and open a new shell
that does not source `.colcon-formal-v1`. To undo the later activity-source
merge, use the targeted 2026-09-02 `RESTORE.md` named at the top; do not remove
or rewrite paths manually. Disaster recovery is only via
`/home/lpc/workspace/arena5_ws_archives/20260829_164103_full_workspace_pre_log_cleanup/RESTORE.md`;
rename the current workspace before extraction and never overlay-extract it.

Known V1 constraints: HuNav reset affects all agents and creates at most one
visible compute-beat switch delay, which is acceptable only for the fixed
single-human scenario. Before adding people, redesign per-agent dynamic tree
switching and re-evaluate transaction/reset semantics. Visibility is only
distance plus human yaw/FOV, with no occlusion or ray tracing.

## Prior deployment task (historical)

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
`/home/lpc/workspace/arena5_ws/logs/runs/20260828_160829_six_behaviors_gpu3`.
It contains a 560585-byte
WebRTC frame, ROS logs, a Nav2 goal success, the six-behavior marker, and
odom/TF/sensor evidence. It continued running for approximately 5 h 46 min at
about 14 Hz HuNav compute and 4.9 Hz display before a clean shutdown. A separate
approximately ten-minute run is
`logs/runs/20260828_155530_six_behaviors_gpu3`; that shorter run was removed by
the later log cleanup and is now recoverable only from the full workspace
archive named in the formal-social section above.

## Build and verification commands

The legacy deployment was once built with full-workspace and in-place colcon
commands. Those commands are intentionally not repeated as executable
instructions here: **do not rebuild into the activity workspace's shared
`build/install/log` for the formal-social feature**. Use only the isolated
`.colcon-formal-v1` commands at the top of this document; consult Git history or
the disaster-recovery archive solely when maintaining the legacy deployment as
a separate task.

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

There is no hard deployment or chassis blocker. The historical deployment
baseline includes a complete D6 velocity/collision suite and navigation plus
six-behavior regression. For the 2026-09-02 source merge, only `18/22` chassis
rows and `0/3` collision cases were run by explicit user direction; see the
top addendum and do not describe that partial rerun as complete. The remaining
items are non-blocking engineering concerns:

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

# Ideal D6 chassis control

Validated on 2026-08-28 with Arena-Rosnav 5.0 and Isaac Sim 5.1 in
`/home/lpc/workspace/arena5_ws`.

## Current status

The ideal D6 chassis is the current default for the workspace launchers. It is
enabled by `scripts/run_common.sh` and `scripts/run_six_behaviors.sh` through:

```bash
ARENA_IDEAL_CHASSIS=true
ARENA_PHYSICS_DT=0.016666666666666666
```

Set `ARENA_IDEAL_CHASSIS=false` explicitly to run the previous wheel-contact
skid-steer model. The older angular gain/static compensation is used only in
that fallback mode and is bypassed in D6 mode.

## Model and control contract

The controller subscribes directly to `/cmd_vel` and implements an ideal
planar velocity-controlled mobile base:

- a world-to-articulation D6 joint locks `transZ`, `rotX`, and `rotY`;
- `transX`, `transY`, and `rotZ` remain free;
- the command is limited to 1.5 m/s linear and 2.0 rad/s angular;
- references are slew-limited to 2.0 m/s^2 and 4.0 rad/s^2;
- a 0.5 s command timeout ramps the references back to zero;
- the resulting body-forward linear velocity and yaw velocity are written to
  the articulation immediately before each fixed 1/60 s PhysX step;
- no pose, transform, or teleport operation is used;
- PhysX performs the subsequent contact and collision solve.

This is intentionally an ideal velocity actuator, not a high-fidelity tyre
model. The root twist is refreshed before every physics step, so sustained
contact can continue to apply commanded effort; the post-step motion remains
collision-limited. The three wall tests below prove that the robot does not
cross the wall even while `/cmd_vel` continues toward it.

Wheel commands and wheel joint states remain available for visualization and
instrumentation. The spawn code also disables wheel `CollisionAPI` prims when
the importer exposes them under the robot hierarchy. The current Jackal import
reports `disabled_wheel_colliders=0`, so the validated velocity and collision
results do not depend on that optional traversal path.

The chassis `PhysxForceAPI` assist used by earlier experiments is not enabled.
There is no point-specific compensation, sign-specific branch, gain tuning,
friction reduction, wheel-geometry tuning, PI loop, pose write, or teleport.

## Actual chassis feedback and odometry

`isaac_utils/graphs/odom.py` uses the official
`isaacsim.core.nodes.IsaacComputeOdometry` node for rigid-body linear and
angular velocity. Those outputs populate `/odom.twist`; pose and TF continue to
come from the articulation transform. `publishRawVelocities=false` keeps the
twist in `child_frame_id=base_link`, as required by `nav_msgs/Odometry`.

The HuNav bridge consumes this `/odom.twist` directly and rotates body-frame
linear velocity into the map/odom frame before constructing `/robot_states`.
Pose differencing and low-pass filtering are no longer the primary feedback
path.

## Isolated velocity test method

`scripts/run_ideal_chassis_matrix_isolated.sh` starts a completely new Isaac
world for every command pair. Each world has no HuNav, no Nav2, one test
publisher, a robot at the clean test start pose, and fixed 1/60 s physics. The
harness records TF-derived chassis velocity, wheel targets/actuals/effort,
sample rates, roll/pitch, and base height. Contact or excessive posture motion
marks a row invalid instead of allowing it into the accuracy result.

Run the full matrix with:

```bash
cd /home/lpc/workspace/arena5_ws
GPU_ID=3 ./scripts/run_ideal_chassis_matrix_isolated.sh \
  logs/chassis_control/<run_name>
```

The authoritative result is
`logs/chassis_control/20260828_d6_isolated_full/results.csv`. All 22 fresh-world
cases are valid. Across the full matrix plus four hold-out points:

- maximum linear mean error: 0.086511% at `(0.6,-0.4)`;
- maximum angular mean error: 0.313368% at `(0.6,-0.4)`;
- maximum linear/angular standard deviation: 0.008307 m/s / 0.012452 rad/s;
- maximum absolute roll/pitch: 0.000286 / 0.003086 rad;
- base Z range: 0.064500--0.064972 m;
- mean TF wall rate: 42.324 Hz;
- simulation-time/controller rate: 60.000 Hz.

The required isolated re-test of `(0.7,-0.6)` is also preserved at
`logs/chassis_control/20260828_d6_resume_07m06_retry/results.csv`; it measured
`(0.699998,-0.599643)`, or 0.000303% / 0.059522% error.

## Before/after velocity comparison

Entries are mean absolute percentage errors. A dash means the corresponding
command component is zero. Baseline evidence is
`logs/chassis_control/20260827_111238_baseline/results.csv`; D6 evidence is the
isolated result above.

| cmd `(v,w)` | baseline `v` % | D6 `v` % | baseline `w` % | D6 `w` % |
| --- | ---: | ---: | ---: | ---: |
| `(0.2,0)` | 1.695 | 0.009 | - | - |
| `(0.5,0)` | 0.518 | 0.002 | - | - |
| `(0.8,0)` | 0.255 | 0.005 | - | - |
| `(0,+0.2)` | - | - | 34.652 | 0.060 |
| `(0,-0.2)` | - | - | 41.023 | 0.060 |
| `(0,+0.4)` | - | - | 11.815 | 0.060 |
| `(0,-0.4)` | - | - | 4.663 | 0.059 |
| `(0,+0.8)` | - | - | 6.614 | 0.060 |
| `(0,-0.8)` | - | - | 5.985 | 0.059 |
| `(0,+1.2)` | - | - | 3.119 | 0.060 |
| `(0.3,+0.4)` | 34.490 | 0.001 | 80.183 | 0.060 |
| `(0.3,-0.4)` | 13.868 | 0.004 | 74.474 | 0.060 |
| `(0.3,+0.8)` | 61.624 | 0.000 | 66.012 | 0.061 |
| `(0.3,-0.8)` | 53.494 | 0.001 | 60.232 | 0.059 |
| `(0.6,+0.4)` | 15.038 | 0.001 | 79.736 | 0.060 |
| `(0.6,-0.4)` | 4.209 | 0.087 | 78.398 | 0.313 |
| `(0.6,+0.8)` | 28.903 | 0.002 | 75.143 | 0.061 |
| `(0.6,-0.8)` | 25.295 | 0.001 | 87.813 | 0.059 |

Hold-out/generalization results were not used to choose gains:

| cmd `(v,w)` | actual mean `(v,w)` | error `(v,w)` % |
| --- | --- | --- |
| `(0.45,+0.6)` | `(0.449995,+0.599637)` | `(0.001045,0.060472)` |
| `(0.45,-0.6)` | `(0.449995,-0.599643)` | `(0.001195,0.059507)` |
| `(0.2,+0.6)` | `(0.199989,+0.599637)` | `(0.005648,0.060502)` |
| `(0.7,-0.6)` | `(0.699995,-0.599643)` | `(0.000694,0.059556)` |

## Collision validation

Run with:

```bash
GPU_ID=3 ./scripts/run_ideal_chassis_collision_suite.sh \
  logs/chassis_control/<run_name>
```

Evidence:
`logs/chassis_control/20260828_d6_collision_suite/results.csv`.

| case | command `(v,w)` | min wall clearance m | contact normal speed mean/median m/s | result |
| --- | --- | ---: | ---: | --- |
| frontal | `(0.8,0)` | 0.278975 | 0.061566 / 0.000002 | no penetration, posture valid |
| oblique | `(0.8,0)` | 0.278979 | 0.063853 / 0.000095 | no penetration, posture valid |
| combined | `(0.6,+0.8)` | 0.278979 | 0.068823 / 0.000216 | no penetration, posture valid |

The nonzero contact means include approach/transient samples; the near-zero
medians and final center positions at approximately x=0.279 m show sustained
PhysX collision limitation. All three rows have `valid=True`,
`no_center_penetration=True`, and `posture_valid=True`.

## Full-system regression

The final default-launch evidence is
`logs/runs/20260828_160829_six_behaviors_gpu3` and was started with only:

```bash
GPU_ID=3 ./scripts/run_six_behaviors.sh
```

The launcher printed `Ideal D6 chassis: true`; no D6 environment override was
needed. Results:

```text
SMOKE_NAVIGATION_OK start=(3.000,3.000) end=(4.811,3.003)
moved=1.811m lidar_messages=84

SIX_BEHAVIORS_VERIFY_OK types=1,2,3,4,5,6 active=3,5
responses=3,4,5,6 robot_distance=0.891 robot_states=87
```

The periodic Curious state can occur before a verifier subscribes. The first
post-navigation verifier invocation missed that transient; a second invocation
within the same unmodified run observed it and produced the required marker.

Measured system data in the final run:

- `/odom`: approximately 21.2 Hz, official rigid-body twist;
- `/lidar`: approximately 3.45 Hz;
- `/lidar/points`: approximately 20.7 Hz;
- `/isaac/joint_states`: approximately 21.4 Hz;
- aggregate `/tf`: approximately 123 Hz;
- `odom -> base_link`: stable Z about 0.0645 m and effectively zero roll/pitch;
- `base_link -> lidar_link`: `[0,0,0.142]`;
- `base_link -> imu_link`: identity;
- WebRTC TCP 49100 and Foxglove TCP 8765 listening;
- `webrtc_frame.png`: 560585 bytes, visually checked with the robot and all six
  Characters present.

A separate approximately ten-minute D6 run is preserved at
`logs/runs/20260828_155530_six_behaviors_gpu3`; its HuNav bridge remained near
14.1 Hz compute and 4.9 Hz display without a structural fault.

The final default-launch instance itself was then left running from about
16:08 to 21:55 CST (approximately 5 h 46 min). HuNav continued reporting near
14 Hz compute and 4.9 Hz display, after which `Ctrl-C` shut down all Isaac,
Nav2, HuNav, and Foxglove children cleanly. No workspace runtime process,
Isaac service, TCP 49100 listener, or TCP 8765 listener remained.

## Backup and recovery

The baseline taken immediately before restoring D6 is:

```text
backups/20260828_151525_pre_d6_resume/
```

Read its `RESTORE.md` before using it. The older complete runnable archive is:

```text
backups/20260827_110822_chassis_control_baseline/arena5_ws_runnable.tar.zst
SHA256 714e80af77a9ffe6362ede352104dd8221a44bfcbe233d3ce04d89b39abe50d8
```

The source candidate originally resumed for this work remains at:

```text
backups/20260828_112347_d6_candidate_archived/
```

The finalized D6 source/build/install snapshot and exact restore commands are
stored under `backups/20260828_162000_d6_final/`.

Before any recovery, stop Arena with `Ctrl-C` or use the workspace-scoped
`scripts/cleanup.sh`. Never mix source from one snapshot with build/install
trees from another.

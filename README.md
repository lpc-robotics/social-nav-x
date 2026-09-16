# arena5 MPC development workspace

This repository contains the isolated development, evidence, and eventual additive release artifacts for migrating the MPC core from `/home/lpc/MPC-Navigation` to `/home/lpc/workspace/arena5_ws`.

The stable workspace is a read-only underlay during P0-P5. Existing files in it must not be edited, rebuilt, reset, or cleaned. The original DWB entry remains the default.

Current phase: P0 through P6 passed. The current immutable release is installed at
`/home/lpc/workspace/arena5_ws/optional/mpc/releases/20260916-8329398`, and the
only new stable-workspace entry point is
`/home/lpc/workspace/arena5_ws/scripts/run_six_behaviors_mpc.sh`.  The original
`run_six_behaviors.sh` remains the default DWB entry and retains its protected
hash.

Phase evidence is under `evidence/p0` through `evidence/p6`.  P1 contains the
locked CasADi SDK evidence, independently implemented `arena_mpc_core`,
Python/C++ numerical comparisons, and capacity benchmarks.  P2-P4 cover Nav2
integration, fault handling, static navigation, and dynamic-human safety.  P5
contains the maximum-load benchmark, paired DWB/MPC comparison, and accepted
30-minute endurance run. P6 records the relocatable release audit and final
MPC/DWB rollback smoke. `evidence/visualization` records the later additive
Foxglove visualization release, and `evidence/costmap_fix` records the global
costmap policy correction and release rollback drill. All earlier immutable
releases remain available for rollback.

Run the released controller from the stable workspace with:

```bash
cd /home/lpc/workspace/arena5_ws
GPU_ID=3 ./scripts/run_six_behaviors_mpc.sh
```

Run the unchanged DWB default with:

```bash
cd /home/lpc/workspace/arena5_ws
GPU_ID=3 ./scripts/run_six_behaviors.sh
```

The MPC global costmap intentionally uses only `StaticLayer` and
`InflationLayer`. The local `VoxelLayer` keeps raw `/lidar` as its obstacle
source and additionally consumes `/lidar_clearing` with `marking=false` and
`clearing=true`. The clearing topic is derived from conservative render depth;
it does not rewrite ambiguous RTX `-1`, `0`, or NaN scan bins. Dynamic people
remain direct `/human_states` inputs to MPC.

The local clearing source is enabled by default in release
`20260916-8329398`. To disable it for the next launch without modifying the
release, set `ARENA_DEPTH_CLEARING=false`. To restore the complete preceding
version, use:

```bash
cd /home/lpc/workspace/arena5_mpc_ws
./scripts/select_mpc_release.sh 20260915-bf2bc7c
```

Release selection verifies every file in the target release, the protected
stable DWB underlay, and overlay package resolution before atomically changing
the MPC wrapper. To roll back, select any retained immutable release, for
example:

```bash
cd /home/lpc/workspace/arena5_mpc_ws
./scripts/select_mpc_release.sh 20260915-df9a55d
```

This changes only the next launch; it does not stop or replace an already
running process.

The governing plan is `ARENA5_MPC_MIGRATION_PLAN_REV2.md` in this repository.

## Foxglove visualization

The MPC launch starts both `foxglove_bridge` and the read-only
`mpc_visualizer` by default. The visualizer is outside the command path and
does not publish velocity commands. It exposes these Foxglove/RViz-friendly
topics:

| Topic | Type | Content |
|---|---|---|
| `/mpc/global_plan` | `nav_msgs/msg/Path` | Latest Nav2 global plan; transient-local so a new viewer receives the latest plan. |
| `/mpc/local_trajectory` | `nav_msgs/msg/Path` | The controller's current MPC prediction, normally 26 poses for `N=25`. |
| `/mpc/human_markers` | `visualization_msgs/msg/MarkerArray` | Human bodies, velocity arrows, constant-velocity predictions, HuNav goals, behavior labels, and the MPC exclusion envelope. |

The visualizer normalizes every `PoseStamped.header.frame_id` to the enclosing
Path frame when the pose frame is empty. This handles the installed Navfn
planner's `Path.header.frame_id=map` output without exposing an empty frame to
Foxglove. A non-empty pose frame that conflicts with the Path header is
rejected instead of being relabeled.

In a Foxglove 3D panel, set the display frame to `map` and enable those three
topics. Path colors are viewer settings. Human marker colors encode the HuNav
behavior: blue regular, gray impassive, yellow surprised, purple scared,
turquoise curious, and red threatening. The tall opaque cylinder is the human
body. The flat translucent cylinder is the center-to-center exclusion area
used to explain the configured `0.35 m` MPC clearance; it includes the human
radius, the Jackal circumscribed radius, and the `0.05 m` geometry allowance.

The default MPC Foxglove port is `8775`. From another Windows machine,
`localhost` refers to Windows itself. Bind the server listener explicitly and
connect Foxglove to the Linux server address:

```bash
cd /home/lpc/workspace/arena5_ws
ARENA_FOXGLOVE_ADDRESS=0.0.0.0 GPU_ID=3 ./scripts/run_six_behaviors_mpc.sh
```

For this server the current endpoint is `ws://10.16.205.165:8775`; use the
server's current IP if it changes, and ensure TCP port `8775` is reachable from
Windows. Set `MPC_VISUALIZATION=false` only when the extra display topics are
not wanted. This does not disable the Foxglove bridge itself.

# P0 runtime parameters — 2026-09-07

Run: stable DWB `20260907_164616_six_behaviors_gpu3` on ROS domain 51.

All queried lifecycle nodes (`map_server`, `controller_server`, `planner_server`, `bt_navigator`, `velocity_smoother`) reported `active [3]`.

| Node/parameter | Actual runtime value |
|---|---|
| `/controller_server controller_frequency` | 10.0 Hz |
| `/controller_server failure_tolerance` | 0.0 s |
| `/controller_server controller_plugins` | `[FollowPath]` |
| `/controller_server FollowPath.plugin` | `dwb_core::DWBLocalPlanner` |
| local costmap footprint | `[[0.1,0.1],[0.1,-0.1],[-0.1,-0.1],[-0.1,0.1]]` |
| local costmap footprint padding | 0.0099999998 m |
| local voxel lidar expected update rate | 0.0 s |
| global costmap footprint | same ±0.1 m square |
| global costmap footprint padding | 0.0099999998 m |
| global obstacle lidar expected update rate | 0.0 s |
| `/planner_server GridBased.plugin` | `nav2_navfn_planner/NavfnPlanner` |
| `/planner_server GridBased.allow_unknown` | true |
| `/velocity_smoother smoothing_frequency` | 20.0 Hz |
| `/velocity_smoother feedback` | OPEN_LOOP |
| `/velocity_smoother velocity_timeout` | 1.0 s |

The first stable DWB navigation verification passed:

```text
SMOKE_NAVIGATION_OK start=(3.000,3.000) end=(4.784,2.974) moved=1.784m lidar_messages=130
```

Because lidar's observed wall-clock p99 interval was 0.510958 s while its simulation timestamp interval was 0.1 s, the MPC initial lidar wall timeout is revised from the generic 0.3 s minimum to `3 * p99 = 1.532875 s`, rounded up to 1.55 s for the P2 starting configuration. ROS-time age remains limited to 0.3 s. HuNav's corresponding initial wall timeout remains 0.6 s (`3 * 0.199503 s`, rounded up).

# P0 baseline results — 2026-09-07

Status: PASS.

## DWB navigation smoke

Each run used the unmodified stable command on GPU 3 and a fresh process start. The installed `verify_runtime` sent the fixed `(5.0, 3.0, yaw=0)` NavigateToPose goal from the initial `(3.0, 3.0)` pose.

| Trial | Stable run directory | Result | End pose | Displacement | Lidar samples seen by verifier |
|---|---|---|---|---:|---:|
| 1 | `20260907_164616_six_behaviors_gpu3` | PASS | `(4.784, 2.974)` | 1.784 m | 130 |
| 2 | `20260907_165524_six_behaviors_gpu3` | PASS | `(4.846, 2.836)` | 1.853 m | 95 |
| 3 | `20260907_165717_six_behaviors_gpu3` | PASS | `(4.852, 2.834)` | 1.859 m | 108 |
| 4 | `20260907_165908_six_behaviors_gpu3` | PASS | `(4.775, 3.042)` | 1.775 m | 122 |
| 5 | `20260907_170111_six_behaviors_gpu3` | PASS | `(4.775, 3.042)` | 1.775 m | 122 |

All five actions reported success and all displacements exceeded 1 m. Each launch was stopped with SIGINT after verification and its child processes exited cleanly.

## HuNav six-behavior baseline

Run directory: `20260907_203342_six_behaviors_gpu3`.

The stable stack was launched with `NAVIGATION=false`. The first verifier attempt began before Isaac's roughly 60-second cold start completed and timed out at readiness without publishing motion. After the same stack reported `SIX_BEHAVIORS_READY`, the unmodified verifier passed:

```text
SIX_BEHAVIORS_VERIFY_OK types=1,2,3,4,5,6 active=3,5 responses=3,4,5,6 robot_distance=0.900 robot_states=346
```

This verifies all six configured behavior types and the special responses expected by the stable verifier without a concurrent Nav2 publisher.

## Runtime and visualization evidence

- `map_server`, `controller_server`, `planner_server`, `bt_navigator`, and `velocity_smoother` were active during the navigation run.
- WebRTC screenshot artifacts exist for all five navigation runs and the behavior run; sizes ranged from 430214 to 547334 bytes.
- Isaac logs reported `Streaming server started`; the HuNav bridge reported `SIX_BEHAVIORS_READY`.
- Foxglove bridge started and advertised runtime topics during the runs.
- The 60-second topic/QoS observation and runtime parameters are stored beside this file.

## Post-run protection and cleanup

- The protected stable hashes exactly matched the preflight values after P0.
- Expected stable ports had no remaining listener.
- No Arena/Isaac/Nav2/HuNav process owned by these runs remained.
- GPU 3 returned to its pre-existing external load (about 6031 MiB used); no external process was stopped or modified.

P0 gate is closed. P1 may start in the isolated workspace.

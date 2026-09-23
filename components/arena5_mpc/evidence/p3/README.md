# P3 navigation and static-obstacle gate

P3 passed on 2026-09-08 in the isolated `feature/mpc-nav2` workspace.  Every
final scenario used a fresh Isaac/Nav2 process, a private ROS domain, the MPC
command chain, the unified 0.48 m x 0.44 m footprint, and the final 0.025 m per
0.1 s path-reference progression.  The protected underlay hashes passed before
every launch.

| Scenario | Result | Worst successful goal error | Maximum simulation time | Static clearance |
|---|---:|---:|---:|---:|
| `straight.json` | 5/5 success | 0.2211 m / 0.0017 rad | 4.167 s | no test wall |
| `turn.json` | 5/5 success | 0.0406 m / 0.2349 rad | 7.317 s | no test wall |
| `orientation.json` | 5/5 success | 0.0155 m / 0.2311 rad | 4.567 s | no test wall |
| `wall_edge.json` | 5/5 success | 0.0597 m / 0.2305 rad | 9.034 s | 0.2250 m |
| `narrow.json` | 5/5 success | 0.2100 m / 0.0649 rad | 4.450 s | 0.1835 m |
| `corner.json` | 5/5 success | 0.2066 m / 0.2170 rad | 8.067 s | 0.3324 m |
| `small_obstacle.json` | 5/5 success | 0.0616 m / 0.2368 rad | 18.151 s | 0.3037 m |
| `rotation_sweep.json` | 5/5 success | 0.0155 m / 0.2302 rad | 4.867 s | 0.1114 m |
| `unknown.json` | 5/5 expected abort | no motion or command rebound | 0.317 s | no test wall |
| `blocked.json` | 5/5 expected abort | no motion or command rebound | 0.250 s | 0.3600 m |

All reachable scenarios stayed within the configured 0.25 m / 0.25 rad goal
tolerances and the 150 s per-action gate.  All wall scenarios recorded the
spawned geometry in the global costmap.  The exact oriented robot-polygon to
wall-polygon monitor saw zero collision samples in every scenario.  Unknown and
fully blocked goals aborted and produced no nonzero `/cmd_vel` rebound during
the post-failure observation window.

P3 exposed four assumptions that were corrected without changing the
architecture or relaxing a gate:

- The original 0.05 m reference progression represented 0.5 m/s at `dt=0.1`,
  above the 0.26 m/s model limit, and caused sharp-corner cutting.  The final
  value is 0.025 m, representing a reachable 0.25 m/s time-indexed reference.
- The first corner fixture combined a narrow U-shaped pocket with constrained
  rotation.  It was replaced by a fixed L-shaped obstacle with explicit grid
  clearance; the rejected fixture and log are retained as
  `corner_rejected_u_shape*`.
- Lidar cannot observe five collinear short obstacles simultaneously before
  motion.  The final test records whether each wall was seen at least once as
  the robot advances, and waits for a fresh first post-spawn costmap frame.
  Rejected pre-visibility, post-pause freshness, and insufficient reaction
  distance trials are retained as `small_obstacle_rejected_*`.
- A rotation fixture with only about 0.067 m continuous clearance at the first
  rejected predicted pose lay inside the 0.05 m costmap discretization and
  measurement margin.  The final fixture has at least 0.111 m measured
  clearance; the conservative rejection and diagnosed collision pose are
  retained as `rotation_sweep_*` rejected evidence.

The action result can precede the last 20 Hz velocity-smoother output.  Final
goal error is therefore measured from odometry after a 0.35 s settling window;
the rejected result-time sample is retained as `wall_edge_rejected_result_settle*`.

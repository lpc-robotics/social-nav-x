# P2 interface, timing, and fault gate

P2 passed on 2026-09-08 in the isolated `feature/mpc-nav2` workspace.  All
runtime tests used private ROS domains and only processes started by the test.
The protected underlay hashes passed before every launch.

| Evidence | Verified behavior |
|---|---|
| `basic_navigation_cross_boundary.json` | Controller plugin executes in `controller_server`; navigation succeeds; full command path p95 63.82 ms, p99/max 69.37 ms, 0% over 100 ms. |
| `topic_tf_graph.json` | The watchdog is the only `/cmd_vel` publisher; one `/odom` source; TF has no child with multiple parents. |
| `speed_limit.json` | Runtime speed limit 0.08 m/s is honored (observed maximum 0.079973 m/s). |
| `goal_orientation.json` | Final yaw error 0.2259 rad, within the configured 0.25 rad tolerance. |
| `non_unit_tf.json` | Navigation succeeds with a non-identity map-to-odom transform. |
| `path_preemption.json` | A replacement path increments generation; the old action aborts and the replacement succeeds. |
| `cancel.json` | Cancel returns CANCELED and zero output begins within 59.7 ms without rebound. |
| `controller_deactivate.json` | Controller lifecycle deactivation begins sustained zero output within 240.7 ms. |
| `costmap_deactivate.json` | Costmap loss is reported as `costmap_input`; sustained zero begins within 160.0 ms. |
| `pause_input_timeout.json` | Simulation pause produces `odom_input`; zero begins 394.0 ms after pause acknowledgement and remains zero; the stopped robot is confirmed after unpause. |
| `human_outage.json` | Empty-state publisher outage produces `human_input`; zero begins 596.9 ms after acknowledgement, without rebound; navigation recovers. |
| `human_stale_timestamp.json` | A 10 s old human sample advances reset epoch, produces zero in 19.5 ms, cannot poison the cache, and navigation recovers. |
| `human_future_timestamp.json` | A 10 s future human sample advances reset epoch, produces zero in 19.2 ms, cannot poison the cache, and navigation recovers. |
| `hunav_process_pause_late_response.json` | The real six-behavior bridge paused for 0.75 s; `human_input` zero began in 600.5 ms, no nonzero command rebounded while paused, the first delayed response was 0.2333 s old, and output recovered 127.5 ms after the first valid human state. |
| `clock_rollback.json` | Injected 5 s `/clock` rollback produces zero and a permanent reset latch in about 20 ms; restart is required. |
| `solver_failure_overlap.json` | Deliberate initial overlap drives IPOPT to its wall limit; no unsafe result is accepted, FollowPath aborts, and output remains zero. |
| `watchdog_lidar_cutoff.json` | With all other leases healthy, stopping raw lidar alone produces `lidar_input` and zero in 280 ms. |

The six-behavior process test deliberately gates on safe stop and command
recovery, then cancels the action.  Long-duration human-navigation success is
part of P4, not the P2 interface gate.

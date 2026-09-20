# Zero-command online fault evidence

This directory records the investigation of the goal
`(8.1906501, 5.7491006)` in the six-behavior HuNav scenario. A probe timeout
cancels its own action and reports action status `0`; it is not a Nav2
`ABORTED` result.

| Evidence | Implementation | Result |
|---|---|---|
| `live_goal.json` | release `20260919-5d32739`, first run | Moved 3.143 m, then repeatedly waited on solver timeout/infeasible near the conservative human-clearance boundary. |
| `live_goal_retry.json` | same release, retry from the partial endpoint | Moved another 1.680 m and entered direct-away `clearance_recovery`; the threatening pedestrian followed, so the recovery direction could keep taking the robot away from the goal. |
| `post_fix_dev_goal.json` | development overlay with 85/95 ms experimental budgets | Moved 4.115 m but missed the 90 ms release contract frequently. The budget experiment was rejected and the configuration was restored to 75/90 ms. |
| `post_goal_biased_recovery.json` | 75/90 ms development overlay with goal-biased recovery and independently feasible iterate acceptance | Moved continuously from `(3,3)` to `(7.6857,5.4257)`, 5.2764 m total. Minimum sampled oriented-footprint clearance was 0.4215 m; full command p95/p99 was 82.79/84.32 ms with no sample over 100 ms. The 420 s wall timeout expired with the controller still in `track` about 0.600 m from the goal, so the probe canceled the action. This proves the persistent-zero lock was removed but is not the final arrival gate. |
| `post_goal_biased_recovery_retry.json` | same development process, same goal from the remaining 0.60 m | Returned `STATUS_SUCCEEDED=4` in 38.61 s wall / 11.57 s odom time, with 0.0398 m final error, 0.4613 m minimum sampled clearance, and 83.98 ms full-command p99. |
| `post_goal_biased_recovery_clean_full.json` | fresh process and reset six-behavior scene, one uninterrupted action from `(3,3)` | **Accepted final development gate:** `STATUS_SUCCEEDED=4` in 311.01 s wall / 90.48 s odom time, 5.8368 m displacement, 0.1325 m final error, 0.4018 m minimum sampled clearance, 84.82 ms full-command p99, and zero samples over 100 ms. The action remained active through solver, costmap, and human waits and was never aborted. |

The reproduced geometric mismatch was:

- robot pose `(5.7315, 6.0521, 2.3493)`;
- human 3 pose `(6.5265, 6.8227)`;
- center distance `1.1072 m`;
- conservative NLP circle threshold `1.1256 m`, giving `-0.0184 m` clearance;
- oriented rectangular footprint clearance about `0.437 m`, above the normal
  `0.35 m` requirement.

The accepted correction keeps the normal and emergency clearance thresholds at
0.35 m and 0.30 m. It permits at most 0.05 m of pre-existing conservative-circle
mismatch, requires future circle clearance not to worsen, turns before translating
when necessary, and chooses the path-biased recovery direction with at least a
0.10 outward cosine. A finite IPOPT time-limit iterate is usable only after the
independent C++ constraint evaluator passes it; the controller still applies
oriented-footprint HuNav checks, full braking checks, costmap checks, latest-input
checks, path/reset generations, the 90 ms commit gate, and the watchdog.

The development overlay now satisfies the clean single-action arrival gate.
After packaging, the selected immutable release must repeat the same goal (or
the release smoke equivalent) before the release wrapper is considered closed.

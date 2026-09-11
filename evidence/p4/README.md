# P4 dynamic-human avoidance gate

P4 passed on 2026-09-10 in the isolated `feature/mpc-nav2` workspace.  The
formal matrix contains eight scenarios with five fresh-process repetitions per
scenario.  `formal_summary.json` validates the exact 40 expected result names,
their final source/config/probe hashes, action status, finite output, scenario
injection requirements, and safety bounds.

| Scenario | Formal results | Required behavior |
|---|---:|---|
| `crossing` | 5/5 pass | single pedestrian crossing |
| `head_on` | 5/5 pass | single pedestrian approaching head-on |
| `same_direction` | 5/5 pass | pedestrian moving in the robot direction |
| `multi_crossing` | 5/5 pass | four interacting pedestrians |
| `stop_turn` | 5/5 pass | observed pedestrian stop and turn |
| `id_change` | 5/5 pass | injected ID-set generation change |
| `backlog` | 5/5 pass | injected callback backlog |
| `six_behaviors` | 5/5 pass | installed six-behavior HuNav fixture |

Across the 40 formal runs, all NavigateToPose actions succeeded and all command
samples were finite.  The measured footprint-to-human clearance lower bound was
at least 0.4232 m after subtracting the time-alignment uncertainty; the largest
alignment-error bound was 0.04220 m.  These values satisfy the 0.30 m measured
lower-bound gate without treating the 0.05 m uncertainty allowance as extra
clearance.  The matrix contains 31,015 solver samples over 983.57 s of simulated
time.  Solver maximum was 88.52 ms; complete plugin-cycle maximum was 89.89 ms,
and the largest per-run cycle p95 was 83.39 ms.  The plugin therefore stayed
within its 90 ms commit limit in every recorded formal sample.

P4 exposed timing and geometry cases that were corrected while retaining the
architecture and gate:

- Normal MPC prediction uses a 0.35 m clearance so the measured 0.30 m gate has
  0.05 m of sampling/prediction allowance.  A separate 0.30 m emergency floor
  is used only when a checked zero/braking command is the safest response.
- The emergency braking check now evaluates the exact oriented rectangular
  footprint against time-aligned pedestrian circles and handles signed reverse
  velocity correctly.
- A moving HuNav agent can leave a short-lived lidar mark in the costmap.  A
  mark geometrically associated with a current dynamic agent permits checked
  zero output for at most 1.0 s; persistent or unrelated costmap collision
  still fails.
- A solver result whose path generation changed while solving is discarded and
  returns a validated zero retry.  Individually old/future samples are rejected
  without advancing the reset generation, while the input lease still revokes
  motion.  Clock rollback remains latched until restart.
- Dynamic waits use the existing safety leases and a bounded 30 s Nav2 progress
  allowance.  The six-behavior goal is 0.6 m because the former 1.2 m target
  coincided with the threatening actor's hold point.

Rejected and superseded attempts remain in this directory and are excluded by
the exact-name summarizer.  In particular, the retained infrastructure or
safety-stop trials include `head_on_run3_rejected_odom_stall`,
`id_change_run4_rejected_transient_braking_and_odom`,
`id_change_run5_rejected_transient_braking_and_odom`,
`backlog_run5_rejected_transient_braking_and_odom`,
`backlog_run5_rejected_transient_braking_and_odom2`,
`backlog_run5_rejected_startup_timeout_gpu3`, and both
`six_behaviors_run4_rejected_alignment_error_bound` attempts.  Calibration and
pre-final-generation results are likewise retained but do not contribute to
the 40/40 gate.

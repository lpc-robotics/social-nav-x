# P5 performance, endurance, and DWB comparison gate

> **2026-09-17 corrective audit:** the timing, stream-health, finite-output and
> DWB/MPC comparison measurements below remain valid, but the original
> endurance pass predicate failed to reject aborted goals. The accepted JSON
> contains 15 successful, 259 aborted and 2 timed-out/cancelled goals. It is
> retained as reproducible historical evidence and is no longer an accepted
> goal-continuity gate. The corrected probe requires zero aborted and zero
> timed-out goals; replacement evidence is recorded under `evidence/abort_fix`.

The performance and paired-comparison portions passed on 2026-09-11 in the
isolated `feature/mpc-nav2` workspace. They combine the protected-underlay
preflight, the maximum fixed-mask benchmark, five fresh-process DWB/MPC pairs,
and one historical 30-minute MPC endurance run whose goal-continuity verdict
was superseded by the corrective audit above.

## Protected underlay preflight

Before P5, the protected installed Nav2 template no longer matched the P0
snapshot because an independent build had copied a pre-existing stable-source
change into the stable install.  No MPC process reset or overwrote it.  The
current installed state was accepted only after a fresh native DWB smoke loaded
`dwb_core::DWBLocalPlanner` and moved 1.770 m.  The development protection
manifest now freezes that observed installed state; full hashes and provenance
are in `preflight/README.md`.

## Maximum-scale solver benchmark

`performance/max_scale_1000.json` validates the fixed-size `N=25`, eight-slot
masked NLP while retaining the 32-dynamic + 128-static input and postcheck
contract.  Each maximum-load condition ran 1000 warm solves:

| Condition | Valid / 1000 | p99 wall time | Expected result |
|---|---:|---:|---|
| feasible | 1000 | 30.963 ms | accepted |
| critical | 1000 | 28.819 ms | accepted |
| infeasible | 0 (1000 bounded timeouts) | 82.101 ms | rejected |

All accepted samples met the numerical residual and geometry checks.  Every
infeasible sample returned a bounded invalid result instead of a command.

## Fresh-process DWB/MPC comparison

`comparison/formal_summary.json` validates ten exact results in the order
`AB, BA, AB, BA, AB`.  Every run used GPU 2, the same custom six-behavior
configuration hash, 1/60 s physics step, 0.26 m/s maximum speed, and a fresh
process.  This HuNav configuration uses explicit custom parameters and has no
random parameter rewrite, so the five repetitions share the same deterministic
initial condition rather than five nonexistent seed parameters.

| Method | Success | Median wall time | Median sim time | Median path | Median command TV | Minimum clearance lower bound |
|---|---:|---:|---:|---:|---:|---:|
| DWB | 5/5 | 8.959 s | 2.133 s | 0.3688 m | 3.1339 | 0.4889 m |
| MPC | 5/5 | 11.166 s | 3.067 s | 0.3712 m | 1.2503 | 0.5371 m |

Each result records the runtime controller plugin, speed limit, local/global
footprint, global costmap layers, GPU snapshot, domain, physics step, and input
configuration hash.  DWB retained its stable ±0.1 m footprint and
static+inflation global costmap.  MPC used the unified ±0.24 m by ±0.22 m
footprint and static+obstacle+inflation global costmap, so this comparison does
not attribute every difference to the controller algorithm alone.

## Formal 30-minute endurance run

`endurance/endurance_30min.json` is the historical report.  It ran on GPU
3 in domain 190 for 1800.529 wall seconds (539.150 simulated seconds), completed
15 successful alternating navigation goals, cancelled two goals after their
180 s per-goal allowance, and recorded 259 aborted goals. Those 261
non-successful results expose the invalid original acceptance rule and cannot
count as successful endurance navigation. The controller and watchdog stayed alive, all output
samples were finite, `/mpc_command_watchdog` was the only `/cmd_vel` publisher,
and no simulation-clock rollback occurred.

The raw controller `Twist` and plugin status are separate headerless topics.
DDS provides no delivery order across them, so a receipt-order-only probe can
pair a status with the next 10 Hz raw command when the raw callback arrives
first.  The final probe matches `ok` status/raw events in either order within
25 ms, requires at least 95% coverage, and adds the measured boundary p99 to
every plugin cycle as a conservative controller-server/publication bound.  The
formal run matched 16,494/16,494 events (100%); callback skew p99 was 1.014 ms.

| Timing item | Formal result | Gate |
|---|---:|---:|
| solver p95 / p99 / max | 81.167 / 83.266 / 89.416 ms | diagnostic |
| plugin cycle p95 / p99 / max | 81.486 / 83.658 / 89.721 ms | 90 ms commit policy |
| complete bound p95 | 82.500 ms | <= 90 ms |
| complete bound p99 | 84.672 ms | <= 100 ms |
| complete bound max | 90.735 ms | diagnostic |
| complete bound over 100 ms | 0 / 16,494 | <= 1% |

The run received 21,565 HuNav, 32,349 odom, 5,408 lidar, and 5,096 raw-costmap
messages.  Maximum wall gaps were 0.480, 0.414, 0.763, and 0.801 s,
respectively, within the frozen sustained-outage limits.  The watchdog still
uses the tighter motion leases and recorded safe stops (`controller_status`,
`command_sequence`, `status_lease`, and one `odom_input`) rather than extending
command validity to the endurance limits.

## Rejected harness and infrastructure attempts

Rejected results remain in place and are excluded from the formal gate:

- the first DWB comparison probe requested an undeclared parameter and received
  a shorter response; the next version queried only the active method;
- the next probe sent a goal before Nav2 lifecycle activation; lifecycle state
  is now gated explicitly;
- one GPU 2 startup entered an Isaac CUDA bad state and produced no lidar;
- the first costmap readiness probe used `nav_msgs/OccupancyGrid` and default
  QoS for `/local_costmap/costmap_raw`; the actual endpoint is
  `nav2_msgs/Costmap`, Reliable, Transient Local;
- one endurance harness turned a single 180 s goal timeout into an entire-run
  failure; cancellation is now verified and the run continues;
- one full-duration run used directional receipt-order matching and an
  undocumented 0.8 s odom sustained-gap threshold.  Its controller remained
  safe, but that report is rejected; it motivated the cross-topic-safe timing
  method and explicit sustained-outage definition used above.

Launch logs are kept locally and ignored by Git.  The structured JSON/CSV
reports contain the synchronized metrics, hashes, runtime parameters, and gate
results needed for reproducibility without recording high-volume image and
lidar rosbag data for every repetition.

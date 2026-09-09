# Phase 4: fixed two-human parallel social automata

Implementation base: `251830d543e99b8666fbb357ca6219712080ea72`.
Branch: `feature/formal-social-automata-phase4`.
Workspace: `/home/lpc/workspace/social-nav-x-formal-v1`.

## Accepted contract

- Exactly one robot (ID 0) and two fixed humans (IDs 1, 2).
- A and B start facing each other at (6, 1.8) and (6, 4.2), with one
  stationary cyclic goal each. SOCIAL uses the existing Regular profile.
- Independent six-state contexts, V1 robot event extraction and five-state
  evaluator; shared pair events evaluated from one immutable input snapshot.
- Mutual near/gaze/stationarity and both agents' readiness must persist for
  1 second before both enter SOCIAL in the same committed transaction.
- SOCIAL priority: personal space > TTC > fast approach > intrusion > pair
  broken > hold. Other five-state transitions retain V1 semantics.
- Peer geometry loss breaks interaction after 0.3 seconds; danger or intrusion
  breaks immediately. Pair reentry cooldown is 1 second.
- Intrusion uses the finite segment between the formation anchors, inflated
  by robot radius plus 0.45 m (enter) / 0.65 m (exit). Consecutive observations
  also check swept segment intersection. No occlusion or prediction is claimed.
- Whole-pair HuNav reset is explicitly accepted: one reset per changed profile
  batch, preserving formal clocks but restarting both internal BTs. No HuNav
  dependency changes or separate single-human SFM managers.
- Default disabled; independent-local mode and shared-events mode are separate.
- Schema 2 batch telemetry, stable shared IDs, replayable inputs, enumerable
  model description for future UPPAAL. V1 API/config/schema/entry stay intact.
- No active-workspace source/install deployment, dependency install, Character,
  D6, Nav2, six-behavior policy, group forces, RL wrapper or PPO changes.

## Protection and build

V1 branch and `.colcon` remain available. Phase 4 output uses
`.colcon-formal-phase4` via `FORMAL_OVERLAY_ROOT`. Only `arena_isaac`,
`arena_humble_compat`, and `formal_social_behavior` are built, using the existing
activity workspace as underlay.

`phase4_baseline.sha256` is checked relative to `/home/lpc/workspace/arena5_ws`.
The historical 20260829 archive manifest remains immutable; it predates four
documented changes (HANDOFF, DEPLOYMENT, main launcher and compatible bridge).
Those activity files byte-match the implementation base. The V1 matrix gains
an explicit manifest override, retaining its historical default and all gates.

## Completion gates

1. Pure core: geometry boundaries, priority, synchronized entry/break, independent
   clocks, rollback, replay, ID permutations, config errors, model enumeration.
2. Mock/real HuNav: ID-based responses, batch reset, both profiles, continuous
   motion/static fields, failures, late calls, concurrency, disabled passthrough.
3. Isolated build/test; all original tests retain their assertions.
4. GPU matrix: formation hold, near without intrusion, central crossing,
   asymmetric intrusion, early fast danger, disabled; two rounds each.
5. Original V1 safe/sudden/fast matrix twice; strict six-behavior and Nav2
   regressions on shared install and Phase 4 overlay.
6. Pre/post guard hashes, manifests and actual evidence; documentation updated.

GPU thresholds: compute >=10 Hz, display >=4.5 Hz, integration step <=0.026 s,
lag <=0.100 s. Surprised speed <=0.02 m/s and facing error <=3 degrees;
Scared has measured outward response. No error, reset storm or partial commit.
Historical chassis rerun remains 18/22 and collision rerun unexecuted, by prior
user direction. This phase does not resume those unrelated suites.

## Verification status

Implemented and accepted; final evidence audit: **2026-09-09**. The independent
three-package overlay, unit/mock/real-HuNav tests, all twelve Phase 4 GPU cases,
six original V1 GPU cases, and shared/overlay six-behavior/Nav2 regressions have
passed. Current-source replay verified **13,466 committed frames**. The 25-file
activity baseline guard still passes. This is a feature-workspace delivery,
not an activity source/shared-install deployment. Detailed evidence follows.

## V1 preservation and feasibility assessment

The authoritative V1 development record and its later shared-install addendum
were read before implementation. V1 already delivers the Event Extractor,
five-state deterministic automaton, complete HuNav profiles, transactional
ResetAgents/ComputeAgents proxy, one-human GPU scenarios and six-behavior/Nav2
regression. Those are reused, not redesigned. In particular the current
six-behavior default is the validated **shared install**, not the older overlay
default described in superseded sections of the handoff.

The proposed Phase 4 is feasible with the following boundaries: SOCIAL is a
discrete interaction label, not a seventh HuNav behavior or a new continuous
motion controller. Fixed facing spawns make formation measurable without adding
an active rendezvous/conversation controller. PEER_VISIBLE is a geometric FOV
observation without ray-cast occlusion. A frozen pair capsule is an explicit
V1 research approximation of social space, not a general group-space model.
Whole-batch reset is an accepted HuNav v1 limitation; it must not be described
as preserving the unmodified peer's internal BT timer.

## State and event contract

| State | Meaning | HuNav type |
| --- | --- | --- |
| NORMAL | Existing V1 baseline/recovery state | Regular (1) |
| ATTENTION | Existing V1 robot attention | Regular (1) |
| CURIOUS | Existing V1 safe-approach response | Curious (5) |
| SURPRISED | Existing V1 surprise; also non-dangerous pair intrusion | Surprised (3) |
| SCARED | Existing V1 personal danger response | Scared (4) |
| SOCIAL | Both ready humans in a committed pair interaction | Regular (1) |

| Event | Scope and exact meaning |
| --- | --- |
| PEER_VISIBLE | Directed A→B / B→A level; disjoint human disks, distance/FOV Schmitt trigger |
| PEER_NEAR | Symmetric pair distance level, enter ≤2.5 m, exit ≥2.8 m |
| MUTUAL_GAZE | Both visible and maximum facing error enters ≤25°, exits ≥35° |
| SOCIAL_SPACE_FORMED | One shared edge after both ready and valid continuously for 1 s |
| SOCIAL_SPACE_BROKEN | One shared edge for intrusion, member danger, or 0.3 s geometry loss |
| ROBOT_INTRUSION | One shared edge per active session when the robot disk enters or sweeps the frozen social capsule |

Visibility enters at distance ≤6 m and absolute bearing ≤100°; it exits at
distance ≥6.5 m or bearing ≥110°. Stationarity enters when both speeds ≤0.05 m/s
and exits when either reaches 0.10 m/s. Formation also requires both local
states NORMAL/ATTENTION, expired local/pair cooldowns, no personal danger,
SUDDEN_NEAR or ROBOT_LOST, and the robot outside old and proposed social spaces.
These thresholds live in the separate `formal_social_multi_automata.yaml`.

### Ordered SOCIAL transition table

Evaluate the first true row, once per agent per input timestamp:

| Priority | Guard | Destination | Recorded cause |
| --- | --- | --- | --- |
| 0 | Personal-space violation | SCARED | PERSONAL_SPACE_VIOLATION |
| 1 | TTC low | SCARED | TTC_LOW |
| 2 | Fast approach | SCARED | ROBOT_FAST_APPROACH |
| 3 | Shared intrusion edge | SURPRISED | ROBOT_INTRUSION |
| 4 | Shared broken edge | NORMAL | SOCIAL_SPACE_BROKEN |
| 5 | Otherwise | SOCIAL | No transition |

The pair coordinator prioritizes intrusion over member danger over sustained
geometry loss. Each human still evaluates its own danger first. Therefore the
same shared intrusion can produce `(SCARED, SURPRISED)`. Danger outside the
capsule produces a broken edge but no intrusion, e.g. `(SCARED, NORMAL)`.
ROBOT_NEAR and ROBOT_LOST alone do not break an active social interaction.
NORMAL/ATTENTION may take synchronized `social.form` only when the shared
readiness protocol passed; all other old-state decisions use the V1 evaluator.

For formation anchors A, B and robot center R, instantaneous entry means
`distance(R, finite_segment(A,B)) <= robot_radius + 0.45 m`; exit is
`>= robot_radius + 0.65 m`. The finite-segment projection is clamped to [0,1],
so endpoints are included. Consecutive observed robot positions define a motion
segment; its minimum distance to the frozen segment detects an intervening
crossing even with both observed endpoints outside. No future trajectory is
predicted. Anchors remain after a break to enforce exit hysteresis, and a new
formation checks the current candidate anchors too. Initial occupancy never
produces a fictional already-formed interaction.

## Composition and ownership

The new `multi_agent` package contains separate `MultiState` and
`MultiAgentAutomatonContext` types. The original `FormalState`, parser, V1
context and transition table remain unchanged. Each agent owns its local
state-entry, safe-since, reentry and last-transition timestamps plus its own
robot-event Schmitt memory. The coordinator owns only bounded pair memory:
visibility/gaze/near latches, formation/break/cooldown clocks, frozen anchors,
session number and previous robot position.

`H_A || H_B` executes as snapshot → robot/peer events → shared pair candidate →
two independent local evaluations → atomic batch commit. Agent IDs are sorted
for semantic evaluation; response order follows the caller. One configured
pair means two local automata plus one small synchronization protocol, rather
than an explicitly materialized 6×6 global transition table. The mathematical
product still exists, and a future model checker may explore it; this design
does not claim to eliminate formal-verification state-space complexity.

The invariant at every successful commit is `pair.active iff both SOCIAL`.
Neither half of a failed service transaction becomes visible. Same-stamp,
value-equivalent requests (canonical field JSON after ID ordering, not CDR
transport bytes) return the cached response;
conflicting requests are rejected. A backward simulation clock creates a new
epoch and clears both local contexts and the pair. There are no wall-clock
behavior guards.

## HuNav transaction and flags

Profiles cover all six local states with complete HuNav fields. SOCIAL,
NORMAL and ATTENTION must have equal Regular profiles, so entering SOCIAL
does not cause a reset. Changed profile signatures cause exactly one
whole-pair ResetAgents followed by ComputeAgents. Only a fully validated
two-human response commits candidate contexts, profiles, counters and trace.
HuNav's unordered response is matched by ID, not array position. Static fields
omitted by the pinned manager are restored using the submitted batch.

A reset/compute failure leaves formal contexts unchanged and marks the backend
dirty; the next accepted new transaction resynchronizes with a batch reset.
A timed-out raw future remains tracked and quarantines new backend operations
until the real operation completes. Its late result is discarded. Local
cancellation is never treated as remote cancellation. Operational reset
attempts/generations and committed reset counts are separately observable.
Telemetry I/O failure after commit is reported but cannot rerun the successful
compute. Thus backend atomicity is recovery-based, not a claim of remotely
rolling back HuNav internals.

Node defaults are `enabled=false`, `shared_events_enabled=false`. The separate
demo explicitly enables both. `enabled=false` passes through to HuNav without
formal traces/reset; `shared_events_enabled=false` runs two independent V1
evaluators without SOCIAL. Flags are startup configuration, not live switches
on an active interaction. Isaac Character, HuNav BT/SFM, six-behavior YAML,
robot chassis and Nav2 settings are not modified.

## ROS and trace interface

The existing ComputeAgents and ResetAgents types are reused. Public compute
remains `/compute_agents`; manager raw services are isolated under
`/formal_social_behavior/multi/{compute_agents_raw,reset_agents_raw}`.
New reliable/volatile depth-10 `std_msgs/String` JSON topics are:

- `/formal_social_behavior/multi/states`
- `/formal_social_behavior/multi/events`
- `/formal_social_behavior/multi/shared_events`
- `/formal_social_behavior/multi/transitions`

Schema 2 records the full immutable input, simulation time, epoch, commit
sequence, agent IDs/names, local contexts/events/metrics, pair memory/geometry,
shared event edges, stable rule IDs, full profiles, changed-profile IDs,
reset-affected IDs, reset reason and backend generation. Pair IDs are sorted;
shared IDs have the form `eE:pA-B:sSESSION:tNS:EVENT`. TTC infinity serializes
as JSON null. The first three topics carry batch frames; transitions and its
JSONL file are emitted only for committed transitions. `steps.jsonl` records
every successful nonduplicate commit. V1 schema 1 topics/files stay unchanged.

`run_manifest.txt` captures source/config hashes, overlay, domain, GPU, flags,
physics settings and extra launch arguments. `model.json` describes the actual
ordered local/pair tables, effective guards excluding higher priorities,
clocks, profile mapping and assumptions. `replay_formal_social_multi` verifies
every semantic field of recorded commits from the input sequence and config.
Backend operational retries are deliberately not part of pure semantic replay.

## Build and acceptance commands

From the feature workspace, with the existing underlay available:

```bash
FORMAL_OVERLAY_ROOT="$PWD/.colcon-formal-phase4" scripts/build_formal_overlay.sh
ROS_DOMAIN_ID=173 FORMAL_OVERLAY_ROOT="$PWD/.colcon-formal-phase4" scripts/test_formal_overlay.sh
GPU_ID=1 ROS_DOMAIN_ID=180 scripts/run_formal_social_multi_demo.sh \
  headless:=true livestream:=false foxglove:=false
```

For matrix/replay tools, source `scripts/env.sh` from the activity workspace
first and then `.colcon-formal-phase4/install/local_setup.bash` from the feature.
The default matrix runs all six scenarios twice in separate ROS domains:

```bash
python scripts/test_formal_multi_simulation_matrix.py --gpu 1 --domain-base 180
ros2 run formal_social_behavior replay_formal_social_multi \
  src/formal_social_behavior/config/formal_social_multi_automata.yaml RUN_DIR/steps.jsonl
FORMAL_OVERLAY_ROOT="$PWD/.colcon-formal-phase4" \
  FORMAL_BASELINE_MANIFEST="$PWD/docs/formal_social_automata/phase4_baseline.sha256" \
  GPU_ID=1 scripts/test_formal_simulation_matrix.sh
```

Unit/service coverage includes all SOCIAL danger-priority combinations,
geometric Schmitt boundaries, swept finite segments, synchronized formation,
danger/break/reentry, immutable contexts, disabled shared protocol, timestamp
rollback/duplicates, ID permutations, malformed batches, reset rejection,
concurrent duplicate requests, late futures, postcommit telemetry failure and
actual HuNav profile/motion/goal behavior. GPU tests additionally require real
odom/human observations, a sole controlled cmd_vel publisher, ten simulation
seconds of initial SOCIAL hold, the physical crossing/asymmetry/fast outcomes,
pre/post fresh runtime counters, exact reset reconciliation and full replay.

The fast case first approaches slowly to x=3.8 m and holds for 0.5 simulation
seconds before the 0.8 m/s command. This puts the robot inside the unchanged
Scared profile's 3 m continuous-action range while still outside the social
capsule. The test requires a fast-approach transition, no intrusion edge and
actual outward velocity; it does not change profile or event thresholds.
Short SCARED responses may recover between lower-frequency `/human_states`
messages, so outward motion is also measured from time-matched committed input
snapshots, including the last SCARED result on a recovery transition.

Shared and overlay baseline regression has its own runner, using the unchanged
original verifiers and fresh launches for six behaviors and navigation:

```bash
python3 scripts/test_formal_multi_baseline_regression.py --gpu 0 --domain-base 210
```

The old shared compat install does not publish `steady_*` fields. Its
independent-window rates are computed from consecutive logged wall timestamps
and compute/update counters, with the same >=10 Hz / >=4.5 Hz thresholds.
No shared bridge update or cumulative-rate substitution is used.

## UPPAAL and training boundary

Stable state/event/rule names, prioritized transition descriptors, nanosecond
clock semantics, pair synchronization and environment assumptions provide the
export boundary. A later exporter must specify the time discretization and
encode pair synchronization as committed/broadcast protocol steps, retaining
the atomic-boundary invariant. Checkable future properties include no partial
SOCIAL commit, deterministic guard selection, no intrusion without a formed
session, safety-priority preservation and conditional recovery liveness.
`uppaal_exported=false` is intentional: no UPPAAL XML/proof has been delivered.
Continuous HuNav/SFM trajectories need a stated abstraction before formal
verification. Trace/schema/config versioning also prepares a future training
observation interface; there is no Isaac RL wrapper, reward design or PPO run.

## Accepted evidence (runs 2026-09-08; audited 2026-09-09)

All paths below are relative to this feature workspace. Initial exploratory
runs and interrupted matrices remain in `logs/` for diagnosis, but are not
counted as accepted evidence.

### Build, unit and service evidence

- Three packages built into `.colcon-formal-phase4`, never shared `install/`.
- Formal: **150/150** tests, including the unchanged original 76 tests and 74
  Phase 4 tests. xUnit reports **335 tests/subtests, zero errors/failures/skips**.
- Compat: **20/20**, zero errors/failures/skips. Character frame conversion:
  **17/17**. Plain pytest and colcon counts differ because xUnit counts subtests.
- Formal/compat xUnit files: `.colcon-formal-phase4/build/{formal_social_behavior,arena_humble_compat}/pytest.xml`.
- Real pinned HuNav tests verify pair formation, one-batch profile reset,
  both Surprised agents' stopping/heading, mixed Scared/Surprised outward motion,
  static goal fields and recovery. Mock tests also verify one-agent changes
  preserve the peer's formal clock while recording both reset-affected IDs.
- Source and installed `multi_agent` Python trees byte-match (excluding
  generated bytecode). No original V1 core, extractor, proxy, configuration,
  scenario verifier, Character, bridge policy or HuNav source was changed.

### Phase 4 GPU matrices

| Scenario | Rounds | Result / key observation |
| --- | --- | --- |
| Formation | 1, 2 | Both SOCIAL; >=10 s hold; zero resets |
| Near, no intrusion | 1, 2 | ROBOT_NEAR while both SOCIAL; zero resets |
| Central crossing | 1, 2 | One shared intrusion; both SURPRISED; robot x>7 m |
| Asymmetric intrusion | 1, 2 | Same edge: A SCARED, B SURPRISED; one initial reset |
| Early fast danger | 1, 2 | Both SCARED before intrusion; outward speed >0.050 m/s |
| Disabled | 1, 2 | Raw Regular motion; no formal trace or reset |

Matrices ran on separate GPUs/domains to avoid shared simulation state:

- `logs/formal_multi_acceptance/20260908_180652_pid2090818/evidence.json`:
  fast, disabled, formation, near ×2; GPU 1, domains 180–187.
- `logs/formal_multi_acceptance/20260908_180328_pid2076555/evidence.json`:
  crossing, asymmetric ×2; GPU 2, domains 188–191.

All **24** before/after windows pass: steady compute **10.236–19.879 Hz**,
display **5.512–5.964 Hz**, maximum integration step **0.025 s**, lag
**0.008–0.025 s**. The after window must be newer than the latest report at
action completion, not merely newer than the pre-action report.

Crossing Surprised facing errors are **1.204–1.259°**, with zero measured speed.
Asymmetric B facing errors are **2.863° / 2.938°**, with zero speed; A's measured
outward speeds are **0.1681 / 0.1674 m/s**. All intrusion edges link the same
stable shared IDs to both agents' transitions. The formation-to-break intervals
in the actual traces all exceed ten simulation seconds.

Crossing retains V1's subsequent SURPRISED → NORMAL → ATTENTION → CURIOUS
rules and ends with three profile resets. Asymmetric cases have **one** reset
at intrusion, then B's ordinary recovery during the post-action window causes
final totals of **3 / 2** resets (round 1 also reaches CURIOUS). These are
trace-accounted profile changes, not duplicate reset calls. Raw reset counts,
committed reset frames and affected IDs reconcile exactly. No partial commit,
service failure or post-cleanup process error occurs in accepted runs.

On 2026-09-09, the current pure evaluator independently replayed all **13,466**
enabled commits across the ten enabled cases and matched every semantic field.
Both disabled runs correctly have no steps file. Config/model hashes match the
current descriptors. Runtime model hash:

```text
3ba2383902d855b72560c76e93951779284ab865c228da252fbca9310853eb33
```

### V1 and baseline regression

- Original V1 matrix:
  `logs/formal_acceptance/20260908_175122_748564110_pid2029126/matrix.log`.
  It reports `FORMAL_SOCIAL_ACCEPTANCE_MATRIX_OK cases=6 rounds=2 scenarios=safe,sudden,fast`;
  all six cases preserve two resets and `regular_motion=stopped`, and all twelve
  original runtime windows pass. Original verifier assertions remain unchanged.
- Shared/overlay baseline:
  `logs/formal_multi_baseline/20260908_180346_pid2078555/evidence.json`.
  Four fresh runs pass original six-behavior or Nav2 verifiers with validated
  package prefixes. Both six-behavior runs observe types 1–6 and responses
  3/4/5/6. Shared/overlay Nav2 goals succeed with measured displacement
  **1.763 / 1.784 m**, respectively. All eight independent runtime windows pass.
- Each accepted matrix reconciles pre/post protection checks. The current
  `phase4_baseline.sha256` check also passed on 2026-09-09. Owned test managers,
  matrix processes and Isaac launch processes have exited; unrelated GPU jobs
  are not modified or stopped.

### Evidence checksums

| Artifact | SHA-256 |
| --- | --- |
| GPU 1 matrix evidence | `fa51df3934085a6fa13c96426c3aac94c879c1a8243168e0868d27d291d6acc3` |
| GPU 2 matrix evidence | `e2bb2cac32cd0d4cb40af784700f9d70e51e8b4bdee30309dab1166f1fe42e35` |
| Shared/overlay baseline evidence | `8d87f6ea033a362d380736f020b88fdd0afa2dde715b5a9cfdb85ccfb9015ecf` |
| V1 matrix log | `ffe01a003986b7350631e46a1519717f7f6ac9a1002c6a6803db0f7a0e7e36bb` |

Engineering issues found during verification were addressed in the feature:
ROS parameter YAML aliases were expanded; raw-response order is matched by ID;
the test client has a separate callback group to observe real late-call
quarantine; duplicate-request keys use canonical full field values instead of
transport serialization; shutdown signals are forwarded once through launch;
the fast test respects the existing HuNav activation range. None of these
changes alter the protected six-behavior baseline or the V1 automaton rules.

# Activity workspace formal-source merge evidence

Date: 2026-09-02

Target: `/home/lpc/workspace/arena5_ws`

Feature source: `/home/lpc/workspace/social-nav-x-formal-v1`
Base: `51ab117dedf6a8173c1704f0edd8d01c7938fb8e`

## Result

The formal-social package, compatible bridge changes, run/test scripts and the
Isaac Character frame conversion were copied into the activity source tree.
Build, install and log output remains isolated under
`/home/lpc/workspace/arena5_ws/.colcon-formal-v1`; the shared install and Conda
environment were not rebuilt or modified.

The activity workspace did have the same visual-heading defect. Its pre-merge
`Person.py` SHA-256 was
`883c1aaec2c242521015313c57f8ce562d5a6122ada27b6c4bcb6a58615df684`,
it passed ROS quaternions directly to the Character Graph, and it had no
`character_frames.py`. ROS uses local `+X` as planar forward while the selected
Isaac People assets render local `-Y` as forward. The merged code applies:

```text
q_character = q_ros * qz(+pi/2)
q_ros       = q_character * qz(-pi/2)
```

This keeps `/human_states` and HuNav in ROS semantics and confines the asset
offset to the Character boundary. The activity and feature copies now have the
same hashes:

```text
Person.py            429a25528bb9cc5cd7b16616a6099d0644777e90155e501ff0099c544adfa7f1
character_frames.py  2e018db9ad9c34c8e4cedb057637628d08f6c7dd4be0fe74fe5ac5cf47d01eaa
test_character_frames.py
                     53d577dcd224ed2b5295ed001dc2e8f208c926ace2436158c03771701a315233
```

## Build and tests

Commands:

```bash
cd /home/lpc/workspace/arena5_ws
env ARENA_BASE_WS="$PWD" FORMAL_OVERLAY_ROOT="$PWD/.colcon-formal-v1" \
  scripts/build_formal_overlay.sh
env ARENA_BASE_WS="$PWD" FORMAL_OVERLAY_ROOT="$PWD/.colcon-formal-v1" \
  scripts/test_formal_overlay.sh
```

Results:

```text
arena_isaac Character frame conversion: 17 passed
arena_humble_compat:                    20 passed
formal_social_behavior:                 76 passed, 2 dependency warnings
xUnit compat: 20 tests, 0 errors, 0 failures, 0 skipped
xUnit formal: 261 tests, 0 errors, 0 failures, 0 skipped
```

The three package prefixes resolve inside `.colcon-formal-v1/install`. The
nested Git index hashes before merge, after build and after tests are identical.
The shared `install/setup.bash`, previously installed bridge/Person module,
Conda history and environment scripts retain their guard hashes. At this
initial merge checkpoint the original six-behavior launcher also retained its
guard hash; the explicitly authorized follow-up below changes only that
launcher so the fixed isolated build is selected at runtime.

The workspace root is not Git. Formal run manifests therefore identify this
layout with:

```text
git_commit=not-a-root-git-worktree
source_revision_kind=merged_content_sha256
source_revision=3ac7a64e30f381221cc0835059ac61c43e86961f603e582625bf58d529e7e2f4
```

## Runtime evidence

### Main six-behavior launcher deployment correction

The activity source and isolated overlay were fixed, but the original main
launcher sourced only the shared install. Consequently the launched executable
was from `install/arena_isaac`, whose installed `Person.py` still has SHA-256
`883c1aaec2c242521015313c57f8ce562d5a6122ada27b6c4bcb6a58615df684`.
This deployment-layer mismatch explains why the formal demo looked correct but
`GPU_ID=3 ./scripts/run_six_behaviors.sh` still showed the 90-degree error.

The main launcher now defaults to `.colcon-formal-v1`, validates the
`arena_isaac` and `arena_humble_compat` prefixes, verifies that installed
`Person.py` byte-matches source, and writes the selected prefixes plus
`person_sha256` to `runtime_manifest.txt`. The implementation commit is
`0359579`; the current launcher SHA-256 is
`24abb1ee73e2ca66aad1c352570c2e1d757fcfadb7ee9f1617c768f54d7f9633`.
The non-launching check is:

```bash
GPU_ID=3 ./scripts/run_six_behaviors.sh --check-overlay-only
```

Main-entry verification on GPU 3 resolved Isaac to
`.colcon-formal-v1/install/arena_isaac/lib/arena_isaac/run_isaacsim` and passed:

```text
SIX_BEHAVIORS_VERIFY_OK types=1,2,3,4,5,6 active=3,5 responses=3,4,5,6 robot_distance=1.020 robot_states=456
compute_hz=14.848--18.399, display_hz=4.806--4.891, max_dt=0.025
```

Evidence and hashes:

```text
/home/lpc/workspace/arena5_ws/logs/regression/six_behavior_overlay_heading_fix_20260902/verification.txt
SHA-256 32b3edcd9b1edfb5208c6f160db202f3efa386f69fa7e553cabc6581b34f691a
/home/lpc/workspace/arena5_ws/logs/runs/20260902_163617_six_behaviors_gpu3/runtime_manifest.txt
SHA-256 6bbe7ad11815d900251568b4155aef509b04d134b24fe07f615031d690e8db29
```

The six behavior types all use this same `Person`/Character Graph boundary.
Regular and Impassive retain their normal navigation; Surprised gets the
correct stationary look-at rendering; Scared, Curious and Threatening retain
their flee/approach/target kinematics while their body visual axis matches ROS.
No per-behavior HuNav algorithm was changed. Character-frame tests pass
`17/17`, and the strict verifier confirms all behavior types plus special
responses.

The targeted pre-change launcher backup is
`/home/lpc/workspace/arena5_ws_archives/20260902_six_behavior_overlay_entry_pre/`.
Its saved launcher SHA-256 is
`bd7459f3dc75cf770cc9985a1d6c5bb7c3aea2c54ddce58b6bd312c2a74077a2`;
follow `RESTORE.md` (SHA-256
`2461d12027ee7b8491417fd34af403368cf7a162e1272f41d04ec7d387e9bc99`)
without resetting nested repositories. The shared install and dependencies are
still unchanged.

Sudden visual scenario:

```text
/home/lpc/workspace/arena5_ws/logs/formal_visual/20260902_110743_494419251_sudden_pid2323558/visual.log
SHA-256 da860f67cb99d76bafc9977842c68491ecad21939fa1c8927838241775fb1e3b
FORMAL_SOCIAL_SCENARIO_OK scenario=sudden ... target=SURPRISED ...
  surprised_speed=0.000000 surprised_facing_error_deg=2.883
```

Original strict six-behavior entry with the merged overlay:

```text
/home/lpc/workspace/arena5_ws/logs/regression/formal_source_merge_20260902/verify_six_behaviors.log
SHA-256 ad0c1a66232befc049c484d818653db68666363a8ce5c5bdc88490229bf235cf
SIX_BEHAVIORS_VERIFY_OK types=1,2,3,4,5,6 active=3,5 responses=3,4,5,6 robot_distance=0.863 robot_states=434
```

This verifies the corrected Surprised stop/facing pose and preserves the
six-behavior compatibility marker. The same boundary conversion is used for
Curious and Scared rendering; it does not alter HuNav goals, velocities or
automaton transitions.

## Explicitly skipped tests

The user explicitly asked to stop and skip the remaining chassis and collision
tests. No process remains running.

- The first chassis attempt stopped at case 1 because `/SpawnUrdf` was not ready
  and produced no measurement result.
- The retry produced 18 of 22 results; all 18 have `valid=true`.
- The remaining 4 of 22 chassis cases were not run.
- The collision suite was not run (`0/3`).

Evidence:

```text
/home/lpc/workspace/arena5_ws/logs/chassis_control/20260902_formal_source_merge_matrix_retry/results.csv
SHA-256 9fc5fc87384de90e11db55965fd213806773ee51298f1a18b9853e890acb4d4a
/home/lpc/workspace/arena5_ws/logs/chassis_control/20260902_formal_source_merge_matrix_retry/results.json
SHA-256 87a093b8e354b61f934e6a89770be9b70a921ee26e7f6ca5d282a01ede484189
```

These partial results must not be described as a complete chassis or collision
pass. The older full deployment baselines remain historical evidence only.

## Rollback

Use only:

```text
/home/lpc/workspace/arena5_ws_archives/20260902_formal_source_merge_pre/RESTORE.md
```

The archive `pre_existing_targets.tar.zst` has SHA-256
`024672330865d7500ee2af9045e0d976bb9dc5f3f8f398ddb96dcb24b9fa6e27`.
The restore procedure moves introduced paths to a retained directory before
extracting pre-existing files. Do not overlay-extract and do not run Git reset
or clean in any nested repository.

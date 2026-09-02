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
Conda history, environment scripts and original six-behavior launcher retain
their guard hashes.

The workspace root is not Git. Formal run manifests therefore identify this
layout with:

```text
git_commit=not-a-root-git-worktree
source_revision_kind=merged_content_sha256
source_revision=3ac7a64e30f381221cc0835059ac61c43e86961f603e582625bf58d529e7e2f4
```

## Runtime evidence

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

# P6 immutable release and rollback gate

P6 passed on 2026-09-12.  It published the additive release
`20260912-58cd661`, exercised that exact release through the stable-workspace
MPC entry, and then exercised the original DWB entry in a fresh ROS domain.
The protected stable-file manifest passed before publication, after each smoke,
and during the final audit.

## Published layout

The immutable 45,067,655-byte release is at
`/home/lpc/workspace/arena5_ws/optional/mpc/releases/20260912-58cd661`.  Its
`RELEASE` file binds controller source and configuration hashes to source commit
`58cd661d8de076a114484757a5b6ef1c4b3521d4`.  `SHA256SUMS` covers every release
file.  The release contains no symlinks and no reference to the development or
temporary build prefixes.

The stable workspace received only additive paths:

- `optional/mpc/releases/20260912-58cd661`;
- `optional/mpc/COLCON_IGNORE`, which prevents an outer colcon scan from
  treating the release as source;
- `scripts/run_six_behaviors_mpc.sh`, a small wrapper bound to this release.

The seven pre-existing protected files still match
`config/stable_protected.sha256`.  In particular, the original
`scripts/run_six_behaviors.sh` hash remains
`9f8d26eae21348e00e6141f95829e51bf5cbc21e0c57d541bd3c8e9df2530836`.

## Relocation and dependency audit

The release was first built with `BUILD_TESTING=OFF` in a temporary prefix,
moved to a different directory, and checked there before any stable-workspace
write.  The relocated benchmark completed, plugin/core/watchdog binaries had no
unresolved dependency after loading the real Arena underlay, and the runtime
check resolved the three MPC packages from the release while resolving Arena
from the stable install.  The same checksum, path scan, runtime, and dependency
checks passed again against the published tree.  `release_audit.json` records
the compact result.

Two earlier release candidates were rejected before stable publication.  The
first retained source/build paths through compiler `__FILE__` strings.  The
second removed those strings but retained colcon's temporary install prefix in
generated POSIX fallback and parent-prefix metadata.  Compiler prefix maps and
metadata relocation fixed both findings; neither rejected candidate wrote into
the stable workspace.

The final development rebuild also passed all five model, solver, Python/C++
reference, and pluginlib tests with zero errors, failures, or skips.  Shell and
Python launch/tool syntax checks passed.

## Released MPC and original DWB smoke

`formal_summary.json` accepts exactly one released MPC result and one native DWB
rollback result.  Both used GPU 3 with 23,516 MiB free at capture, 1/60 s
physics, the same six-behavior input hash, and independent domains 200 and 201.

| Entry | Runtime controller | Action | Position error | Clearance lower bound |
|---|---|---:|---:|---:|
| released MPC | `arena_mpc_controller::MpcController` | success | 0.2248 m | 0.5371 m |
| original DWB | `dwb_core::DWBLocalPlanner` | success | 0.2195 m | 0.5175 m |

The MPC result confirms the unified ±0.24 m by ±0.22 m local/global footprint
and global static+obstacle+inflation layers.  The rollback result confirms DWB
still uses its stable ±0.1 m footprint and global static+inflation layers.  All
commands were finite and each result met the 0.25 m goal tolerance and 0.30 m
conservative human-clearance gate.

Launch logs are retained locally and ignored by Git.  The accepted JSON files
contain the controller identity, effective geometry, safety alignment bounds,
GPU/domain metadata, input hashes, action result, and motion metrics used by the
gate.

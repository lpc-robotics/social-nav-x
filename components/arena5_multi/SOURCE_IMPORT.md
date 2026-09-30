# Multi-robot source import

Imported on 2026-09-30 from `/home/lpc/workspace/arena5_multi_ws`.

- Local branch: `feature/multi-sfm-pedestrians`
- Source commit: `f330dfcd29c6a1f711e3f091798e36852fac742d`
- Development snapshot before package ownership normalization:
  `4dba116d4a1fb2e1b70f33586cef616d88a612d2`
- Publication preparation: normalize package maintainer metadata and update
  the scenario test expectations to the already-delivered speed limits.
- Destination: `social-nav-x/components/arena5_multi/`

This is a snapshot import on top of the existing `social-nav-x` main branch.
The independent local development history and archive tags are not merge
parents or remote branches. Source commit identifiers above refer to the local
archive. The existing single-robot, MPC, and radar components remain available.

Included: tracked `src/`, `config/`, `scripts/`, `baseline/`, `docs/`, compact
`evidence/`, the development plan, README, and ignore rules. Notebook checkpoints
are excluded by the destination repository's ignore rules. Build/install trees,
runtime logs, NvStreamer traces, caches, binary releases, and full local backup
archives are excluded. Vendored LightSFM keeps its license and pinned manifest.

The component requires the stable Arena5 underlay, ROS 2 Humble, and Isaac Sim
5.1 described in its README and baseline manifests. To use this checked-out
component, run its scripts from this directory and set `ARENA_STABLE_WS` to the
prepared underlay (default `/home/lpc/workspace/arena5_ws`). Build the overlay
before using launch or runtime validation commands. The archived-host paths in
historical documentation and evidence describe the original validation host.

Recorded GPU acceptance is historical and scoped to the source digest, speed
limits, and scenarios in each report. Importing this snapshot does not renew
that acceptance or certify all current scenarios. In particular, the original
multi-SFM acceptance used two robots and 1/6 ordinary pedestrians at 0.26 m/s;
later per-scenario speed changes are recorded separately. Keep acceptance
hashes unchanged and rerun the release gate for any new binary release.

## Publication checks

- 280 imported files match the source commit byte-for-byte and retain their
  executable modes; 7 notebook checkpoint files are excluded.
- Scenario/map/Nav2 contracts: 72 checks passed; legacy HuNav profiles: 8 checks
  passed; all 5 pinned LightSFM files matched their manifest.
- Control, pedestrian integration, release-gate, simulation-time, and character
  frame tests: 55 passed (using the existing local ROS environment).
- Parsed 110 Python files and 9 package manifests; checked 10 shell scripts
  with `bash -n`.
- Pre-existing source whitespace and raw HTTP evidence line endings are
  retained to preserve the imported source and evidence bytes.

These publication checks do not include a new GPU simulation or rebuilt
binary-release acceptance run.

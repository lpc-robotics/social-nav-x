# Archive manifest

Archive date: 2026-08-29 (Asia/Shanghai)

Source workspace: `/home/lpc/workspace/arena5_ws`

Target repository: `https://github.com/lpc-robotics/social-nav-x`

## Included directly

- Complete source trees `src/arena-isaac` and `src/arena-rosnav`, excluding
  only their nested `.git`, Python cache, and pytest cache directories.
- Workspace scripts, generated Jackal URDF reference, and current project
  documentation.
- Regenerated `git diff --binary HEAD` patches for all five modified nested
  repositories.
- Compact final D6 validation evidence from
  `backups/20260828_162000_d6_final/evidence`.

## Recreated from pinned upstreams

The following are intentionally not duplicated in this Git repository:

- Arena evaluation, simulation-setup, and tools;
- Foxglove SDK;
- HuNavSim, lightsfm, and people_msgs;
- Navigation2;
- Jackal source.

Their exact origins and commits are in `upstream/manifest.tsv`. The three with
local modifications have matching patches in `patches/`.

## Excluded generated/runtime data

- `.conda`, `.cache`, build/install/log spaces, runtime logs, old backup
  archives, and NvStreamer traces;
- nested Git object databases;
- external Isaac Sim installation and NVIDIA driver/toolkit.

These exclusions contain no unique project source. Deployment prerequisites
and reconstruction commands are documented in `DEPLOYMENT.md` and
`scripts/fetch_upstreams.sh`.

## Key finalized source hashes

```text
b0582da5453ed56bfb54b3d2b58ceec81fe31cfafc72f50e4fe876c903869c2c  src/arena-isaac/arena_isaac/arena_isaac/run_isaacsim.py
8ffc083ea94d2c21a1cf575aa6f98d89b2a14cf84f739dc0165f8f15ced2342f  src/arena-isaac/arena_isaac/arena_isaac/services/SpawnUrdf.py
03a86439bcd9c47bcf2b81553ee8b73b49963aeb09716dd6d74b0c8231881aec  src/arena-isaac/arena_isaac/isaac_utils/graphs/odom.py
8b5d4c17f23eba0d990fcdc4c516bcaf26a06a39db5f777448dcd9d6b27e00ee  src/arena-isaac/arena_humble_compat/arena_humble_compat/hunav_six_behaviors_bridge.py
034ab224ed2fffc13348436e4f57e3d529eb49624d4249e09b9ef40f181b314a  scripts/run_common.sh
bd7459f3dc75cf770cc9985a1d6c5bb7c3aea2c54ddce58b6bd312c2a74077a2  scripts/run_six_behaviors.sh
```

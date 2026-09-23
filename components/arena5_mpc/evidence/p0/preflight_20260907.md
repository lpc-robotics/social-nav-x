# P0 preflight — 2026-09-07

Status: static preflight passed; live DWB baseline not yet completed.

## Stable entry and host

- Stable workspace: `/home/lpc/workspace/arena5_ws`
- Stable entry check: `SIX_BEHAVIORS_RUNTIME_OK mode=shared`
- ROS distribution/domain reported by stable environment: Humble/domain 51
- Isaac path: `/home/lpc/miniforge3/envs/isaaclab/lib/python3.11/site-packages/isaacsim`
- Host: Linux 6.8.0-136-generic x86_64
- Disk at preflight: 218 GiB free on `/home/lpc/workspace`
- Selected baseline GPU: index 3, UUID `GPU-6fc79961-8650-6bf3-08e6-3f1701399bef`
- GPU 3 at preflight: 24564 MiB total, 6029 MiB used, 18053 MiB free, 29% utilization
- Expected stable ports 49100/TCP, 47998/UDP and 8765/TCP had no listener in the host query.
- No active `run_six_behaviors`, `run_isaacsim`, `controller_server`, `bt_navigator`, `velocity_smoother` or `hunav_agent_manager` process was found.

GPU 3 had another user's compute load. It was left untouched. The task authorization permits using any GPU with adequate remaining memory; the selected GPU and its resource state are recorded for the paired DWB/MPC experiments.

## Installed toolchain

| Component | Version/build |
|---|---|
| Python | 3.11.15, conda-forge |
| NumPy | 1.26.4 |
| CMake | 4.4.2 |
| libstdc++ | 16.1.0 |
| Nav2 core/controller/costmap/velocity smoother | 1.1.18, RoboStack `np126py311hbc2a38a_13` |
| DWB core/critics/messages/plugins | 1.1.18, RoboStack `np126py311hbc2a38a_13` |

## Protected file hashes

```text
9f8d26eae21348e00e6141f95829e51bf5cbc21e0c57d541bd3c8e9df2530836  scripts/run_six_behaviors.sh
e69ee213d4be5b91e48954fbd2b633ed183345ddfe90326a9eefe293244a4303  scripts/env.sh
8b5d4c17f23eba0d990fcdc4c516bcaf26a06a39db5f777448dcd9d6b27e00ee  install/arena_humble_compat/lib/python3.11/site-packages/arena_humble_compat/hunav_six_behaviors_bridge.py
04cad2ff6dd166b6e63013c3e925cd7bc1f225c9d52f56f73692397a9cdb7491  config/generated/jackal.urdf
a42dfde54e943ac2d2cfe6b5a4d62953fcfdeb504e931abeb477ab006e82f53f  install/arena_simulation_setup/share/arena_simulation_setup/configs/nav2/nav2.yaml
f94e2ef88cb9113436992e33a461b9638b9676c8caa977c40ae9bd9e2b6d6642  install/arena_simulation_setup/share/arena_simulation_setup/entities/robots/jackal/model_params.yaml
14f482d4e720ad9c070396648ae6d3b32e65d2edf6b2da4c5bbe62b487bfa759  .conda/arena_ros/lib/libcontroller_server_core.so
```

Paths are relative to the stable workspace. These are evidence values, not desired clean-tree values.

## Nested repository state

The stable workspace root is not a Git repository. Existing nested repository state was observed and must be preserved:

| Repository | HEAD | State summary |
|---|---|---|
| `src/arena-isaac` | `16b8e3416517d8c3dc1b5038df4fe11b9a6df46c` | Modified and added Arena Isaac/compat/message files; untracked bridge validation and tests present |
| `src/arena-rosnav` | `c2ff4a87e8686013b53f1e9cd8b01b3ab04fbce4` | Added six-behavior config/launch; package.xml modified |
| `src/arena/evaluation` | `94762429bea19b84cab50a3d0910a736184738a0` | Clean |
| `src/arena/simulation-setup` | `3f142b25d88ce962c803b57cf20f38985d376dea` | Nav2 config and launch modified |
| `src/arena/tools` | `cbf1d05abd436cdb820da9fd0c35847b06a39ae0` | Clean |
| `src/deps/foxglove-sdk` | `05f27efc7e535d9c30c6b0cb4f6aa89de7243870` | Two bridge source files modified |
| `src/deps/hunav/hunav_sim` | `a69cf96d98b0d40e247f819d7aebab661ac68b3b` | `hunav_agent_manager/src/agent_manager.cpp` modified |
| `src/deps/hunav/lightsfm` | `b30327cca189af2fb90443a5d0040cceb46d7195` | Clean |
| `src/deps/hunav/people_msgs` | `0ae47f6e0208cedd84d19d066743fdc1d05fcafa` | Clean |
| `src/deps/nav2/navigation2` | `8e34dcf5790671085a893dc58d2a0940ec80dd1e` | Clean |
| `src/deps/robots/jackal` | `017b8b581a90873047f7d6fe438bd87513be4a76` | Clean |

The full porcelain output was inspected during preflight. No reset, clean, checkout, build, dependency installation, or source edit was performed in the stable workspace.

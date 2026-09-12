# Costmap clearing: isolated development

用户要求继续修复后，已在独立目录恢复工作。原始 RTX 对照实验确认：
空旷、超量程以及小于最小量程的遮挡均可能输出 distance=0、flags=0；
因此不会直接将 LaserScan 的 -1/0/NaN 转换为 +Inf。
开发环境已独立复制并重定位，ROS_DOMAIN_ID 默认为 151，日志和缓存均在本目录。
原目录 11,438 个源文件/配置和 11 个仓库状态再次验证一致。
正在验证能够提供明确几何命中信息的上游清除射线来源。

## Source evidence

Official repository: https://github.com/isaac-sim/IsaacSim

Tag `v5.1.0`, commit `47d886f2858d1ceed556b21c88927aa67bc81c12`.
The reference checkout is `vendor/IsaacSim`, independent of the original
workspace. Relevant files are also preserved in `audit/nvidia/`.

1. [FlatScan initialization and assignment](https://github.com/isaac-sim/IsaacSim/blob/47d886f2858d1ceed556b21c88927aa67bc81c12/source/extensions/isaacsim.sensors.rtx/nodes/OgnIsaacComputeRTXLidarFlatScan.cpp#L259):
   lines 259-268 reset every output range slot to `-1.0f`. Lines 274-290
   compute the azimuth bin and assign each supplied distance. An unfilled
   slot stays `-1`; there is no assertion that a ray was emitted in that
   direction or that the space is free. A supplied negative value would
   also pass through unchanged, so the sentinel is not a unique cause code.
2. [ScanBuffer selection](https://github.com/isaac-sim/IsaacSim/blob/47d886f2858d1ceed556b21c88927aa67bc81c12/source/extensions/isaacsim.sensors.rtx/nodes/OgnIsaacCreateRTXLidarScanBuffer.cpp#L1062):
   indices are selected from the GMO flags; the resulting distance and
   azimuth buffers contain only selected entries. The CUDA predicate in
   [IsaacSimSensorsRTXCuda.cu, lines 55-66](https://github.com/isaac-sim/IsaacSim/blob/47d886f2858d1ceed556b21c88927aa67bc81c12/source/extensions/isaacsim.sensors.rtx/nodes/IsaacSimSensorsRTXCuda.cu#L55)
   tests the `ElementFlags::VALID` bit. Selection discards the reason an
   entry is invalid; FlatScan and LaserScan do not restore it.
3. [ROS publisher](https://github.com/isaac-sim/IsaacSim/blob/47d886f2858d1ceed556b21c88927aa67bc81c12/source/extensions/isaacsim.ros2.bridge/nodes/OgnROS2PublishLaserScan.cpp#L148):
   lines 148-154 copy the depth array to LaserScan ranges without interpreting
   `-1`, `0`, or other exceptional values.
4. `0` is not the FlatScan initialization sentinel. A zero reaching this
   output is a copied distance value. These public source files do not
   define zero as an unambiguous no-hit indication. The ultimate producer
   of the observed zeros is not established by this static investigation.
5. [Isaac Sim 5.1 GMO documentation](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/docs/source/generic_model_output/generic_model_output.html)
   defines sensor-specific validity flags, rather than assigning a unique
   free-space meaning to a LaserScan distance sentinel.
6. [Isaac Sim 5.1 FlatScan documentation](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.sensors.rtx/docs/ogn/OgnIsaacComputeRTXLidarFlatScan.html)
   describes accumulated scan distances and azimuth ordering; it does not
   certify every empty output bin as a measured no-hit ray.

The locally installed `isaacsim.sensors.rtx` extension reports version 15.8.4.
Its Python registration connects `IsaacCreateRTXLidarScanBufferForFlatScan`
to `IsaacComputeRTXLidarFlatScan`. This verifies the relevant pipeline;
it is not a binary identity check against NVIDIA's published C++ sources.

## Root cause: established and unresolved parts

The prior read-only investigation established master costmap occupancy and
missing effective clearing rays. Nav2 only clears along usable observations;
`observation_persistence=0` does not expire already marked cells. Enabling
`inf_is_valid` converts positive infinity, not negative values, zero, or NaN.

This investigation further establishes information loss before `/lidar`:
the current sentinel does not distinguish absent/filtered/bin-missing data
from a confidently measured empty ray. It does NOT prove the origin of
every missing bin or every zero in the running simulator.

Thus a stateless adapter cannot safely infer free space from `-1` or `0`.
Replacing them with infinity could turn missing measurements into fabricated
clearing evidence and erase static obstacles. Keeping them unchanged would
not repair the reported problem. Neither is a validated repair.

## Isolation and Git

Development root: `/home/lpc/workspace/arena5_ws_costmap_fix`.
Root repository and all 11 copied source repositories use
`fix/costmap-clearing`. They have independent `.git` directories; no
worktree or shared Git metadata was added to the original repositories.

Before copying, branch, commit, status, staged diff and unstaged diff were
recorded in `audit/original_repositories.json` and the accompanying patches.
Existing uncommitted source changes were copied, then committed as baseline
snapshots in the development repositories. Their commit IDs are in
`audit/development_baselines.json`; they are preservation commits, not fixes.

Root baseline commit: `89cbc7e`.

Relevant development source baselines:

| Repository | Baseline commit |
| --- | --- |
| `src/arena-isaac` | `c5a1d8e9561b869c7c67898007948460e6d892ec` |
| `src/arena-rosnav` | `39ca4cab709eb29733422c9afd1e6e1503e96a07` |
| `src/arena/simulation-setup` | `06574f395de6807f9df842c7369a228902c87256` |

Source/config changes relative to those baseline commits: **none**.
No revert of a runtime fix is necessary because no such fix was applied.
The development source repositories remain at the baseline commits.

The original `src`, `scripts`, and `config` contents, and all recorded Git
states, are checked by `scripts/verify_original_readonly.py`. The result is
stored in `audit/original_verification.json`. Existing simulator logs or
cache files are outside this content check; no process was launched or
controlled in the original workspace during this task.

## Validation and before/after

| Check | Result |
| --- | --- |
| `-1` is uniquely measured no-hit | Failed source precondition |
| `0` is uniquely measured no-hit | Not established |
| Production adapter/parameter modifications | None |
| Compile or launch candidate | Not performed |
| Pedestrian old cell 254 -> free | No new before/after experiment |
| Inflation trail removal | Not validated |
| Static wall preservation | Not validated by simulation |
| DWB freezing improvement | Not validated |

There is no claim of a successful fix. The evidence does not justify the
requested conversion. The original runtime behavior is unchanged by this task.

## Commands

Inspect the recorded evidence and rerun the read-only integrity check:

```bash
cd ~/workspace/arena5_ws_costmap_fix
git log --oneline
git diff 89cbc7e HEAD
python3 scripts/verify_original_readonly.py
```

No fixed-runtime launch command is supplied because no validated runtime
repair exists. Do not use the copied original launch scripts as a claimed
fixed environment: dependencies were not copied or rebuilt at this stopped stage.

To inspect the untouched source baseline without discarding any later work:

```bash
git -C ~/workspace/arena5_ws_costmap_fix/src/arena-isaac switch --detach c5a1d8e9561b869c7c67898007948460e6d892ec
git -C ~/workspace/arena5_ws_costmap_fix/src/arena-rosnav switch --detach 39ca4cab709eb29733422c9afd1e6e1503e96a07
git -C ~/workspace/arena5_ws_costmap_fix/src/arena/simulation-setup switch --detach 06574f395de6807f9df842c7369a228902c87256
```

Git will refuse conflicting local modifications; do not add `--force`.
Return to the development branch with `git switch fix/costmap-clearing`
in each repository. No command targeting the original repository is needed.

## Prerequisite for a future upstream repair

Capture per-ray status and angular coverage BEFORE the valid-point filter,
then distinguish completed no-hit rays from near-range rejection, missing
samples and invalid returns. If the RTX interface cannot expose that
distinction, obtain independently verified ray intersection results in an
isolated simulator. Only then can a publisher emit positive infinity as
free-space evidence and be tested against pedestrians and static walls.
That upstream implementation was not attempted after the failed adapter
precondition, in accordance with the requested stop condition.

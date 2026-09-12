# Costmap clearing: isolated development

已在独立目录实现可选的渲染深度辅助 clearing 源，原 LaserScan 保留。
不能安全地把 -1/0 转为 +Inf：RTX 原始对照中，空旷、超量程以及近距离遮挡
均可输出 distance=0、flags=0。

## Validated controlled comparison

`tools/test_depth_clearing_scene.py` uses two real installed Nav2 Humble VoxelLayer
masters with identical LaserScan marking input. The stationary sensor observes
a moving box representing an occluder, a static wall, and a below-range occluder.
The candidate receives extra render-depth endpoints, with marking=false.
This controlled test uses synthetic LaserScan marking and real rendered depth;
it is not yet an animated-human or DWB end-to-end benchmark.

| Final region | Baseline | Candidate |
|---|---:|---:|
| Old obstacle lethal cells | 8 | 0 |
| Old obstacle maximum cost | 254 | 0 |
| Inflation cells around old position | 156 | 0 |
| Static wall lethal cells | 20 | 20 |
| Remembered cells behind close occluder | 2 | 2 |

The candidate is clear at the first sample 0.5 simulated seconds after departure.
The baseline remains occupied 5.83 simulated seconds after departure.
Wall marking is intentionally stopped for the last 2.83 simulated seconds:
the retained wall is not an artifact of immediate re-marking.
Two geometry unit tests and isolated compilation pass.
Evidence: `audit/depth_clearing_comparison.json`, master-grid NPY files and
`audit/costmap_before_after.png`. Full Jackal/HuNav scene verification is ongoing.

The new local source is `/lidar_clearing` (PointCloud2), clearing=true,
marking=false, raytrace_max_range=3.0. It is separate from `/lidar` because the
LaserScan cannot identify safe no-hit rays. Four co-located 90-degree depth
cameras preserve geometry occlusion; finite endpoints stop 0.05 m before a hit.
Only explicit positive infinity in the installed depth renderer becomes a
finite range_max endpoint. Zero, negative, NaN and negative infinity are ignored.
No direct LaserScan sentinel conversion and no LiDAR frequency change is made.

Development branch: `fix/costmap-clearing` in the root and source repositories.
Implementation commit: arena-isaac `3d86b95`.
Configuration commit: simulation-setup `388e546`.
Original workspace verification passed again: 11,438 files and 11 Git repositories.

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

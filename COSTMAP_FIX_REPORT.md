# Arena-Rosnav / Isaac Sim / Nav2 动态障碍清除修复

> 本文记录已冻结的 `/lidar` + `/lidar_clearing` 稳定基线。2026-09-16 用户另已验收隔离的 `/lidar_normalized` 单源 Nav2 方案；两条路径的用途、启动、版本保护与回退统一见 [雷达输入交接](RADAR_INPUT_HANDOFF.md)。此处的稳定配置没有被替换。

## 最终根因

Isaac Sim 5.1 的 RTX FlatScan 在输出前把每个 range bin 初始化为 `-1`，随后只用通过有效性筛选的 GMO 元素覆盖对应 bin。因此 `/lidar` 中的 `-1` 表示“这个 FlatScan bin 没有被有效元素填充”，不能证明该方向正常发射且量程内无命中。受控 GMO 实验还显示，空旷方向、超量程表面和低于近裁剪距离的遮挡都可能表现为 `distance=0, flags=0`。`-1`、`0`、NaN 都不具备安全的 no-hit 唯一语义。

Nav2 Humble 的 ObstacleLayer/VoxelLayer 只沿有效 observation ray 清除。`observation_persistence=0` 表示 observation buffer 只保留最新观测，不会按时间老化已经写入 Master Costmap 的障碍单元；`inf_is_valid` 只把 LaserScan 的正无穷解释为量程末端，不能处理 `-1` 或 `0`。所以行人离开旧位置后，Nav2 经常没有获得穿过该位置的 clearing ray，旧的 254 代价继续存在并被 InflationLayer 扩张。

真实场景中，对修复组行人旧位置附近的 20 帧 `/lidar` 取相邻 7 束，共 140 个样本，其中 91 个为 `-1`，49 个为 4.43–6.66 m 的有限值。该结果说明原 LaserScan 在关键方向存在大量缺失 bin，不能作为连续 free-space clearing 证据。

## 修复方案

没有改写 `/lidar` 的任何值，也没有改雷达频率。Isaac Sim 内新增一个可选的渲染深度清除源：

```text
/lidar (LaserScan) ───────────────> VoxelLayer marking + 原 clearing
四个同位 90° 深度视图 -> /lidar_clearing (PointCloud2) -> VoxelLayer clearing only
```

四个 321×3 深度视图覆盖 360°，取中间扫描线，以仿真时间 10 Hz 发布 1284 个 clearing endpoints。有限深度在渲染表面前 0.05 m 截止；安装版 `distance_to_camera` 明确输出的 `+Inf` 才投影到 range max；零、负值、NaN 和 `-Inf` 被忽略。Nav2 中该 PointCloud2 source 使用 `marking=false`、`clearing=true`、`raytrace_max_range=3.0`、`observation_persistence=0.0`，因此它不能新增障碍。

当前实际生成的 global costmap 插件列表只有 `static_layer` 和 `inflation_layer`；虽然 YAML 中保留了 `obstacle_layer` 配置块，它没有列入 `plugins`，不会处理动态观测。本次修复接入正在产生动态残留的 local VoxelLayer。

## 验证结果

### 受控 Isaac 渲染 + 两个真实 Nav2 Humble Master Costmap

两个 Master Costmap 接收完全相同的缺失回波 LaserScan；修复组额外接收渲染深度 PointCloud2。机器人/传感器固定，移动障碍离开，场景另含静态墙和一个遮住记忆障碍的近距离物体。

| 最终旧区域 | 基线 | 修复组 |
| --- | ---: | ---: |
| 致命单元 | 8 | 0 |
| 最大代价 | 254 | 0 |
| inflation 单元 | 156 | 0 |
| 静态墙致命单元 | 20 | 20 |
| 近物体后方记忆单元 | 2 | 2 |

修复组在人离开后第一个采样点（0.5 s 仿真时间）已经清空，基线在 5.83 s 后仍未清除。静态墙的 marking 在结束前 2.83 s 已停止，墙仍保持；近距离遮挡后的单元也保持，验证没有穿透遮挡误清。

### 完整 Jackal + HuNav + Nav2 场景

机器人固定在 `(3, 3)`，位移为 0：

| 场景 | 行人离开后的观测 | 旧位置致命单元 | 最大代价 |
| --- | ---: | ---: | ---: |
| 未启用修复 | 7.50 s | 2 | 254 |
| 启用修复 | 2.00 s | 0 | 0 |

`/lidar_clearing` 实测消息时间为 10.0 Hz，和 `/lidar` 一致；每帧 1284 个有限点，frame 为 `lidar_link`，距离检查全部通过。对比图为 `audit/real_pedestrian_before_after.png`，汇总为 `audit/real_pedestrian_comparison.json`。

DWB freezing 没有直接计数：为了把地图变化与底盘运动分开，真实对照中机器人被固定。造成 freezing 的 Master Costmap 致命残留已经从 254 清为 0，因此其直接触发条件已消除，但仍需在任务级导航运行中统计 recovery/freezing 次数。

四个额外 render products 有性能成本。两次非并发完整场景运行中，基线 compute 约 13.9–14.7 Hz，修复组早期约 8.8–9.5 Hz，display 都约 4.9 Hz；这是粗略数据，不是严格 benchmark。三相机单行优化因未生成有效 clearing 点而被停止并恢复到本报告验证的四相机提交。

## 修改位置

### `src/arena-isaac`

- `arena_isaac/isaac_utils/graphs/sensors/depth_clearing.py`：渲染深度 PointCloud2 publisher、仿真时间节流和清理。
- `arena_isaac/isaac_utils/clearing_geometry.py`：保守 endpoint 过滤和投影。
- `arena_isaac/isaac_utils/graphs/sensors/lidar.py`：按 `ARENA_DEPTH_CLEARING` 创建附加 source，原 `/lidar` 保留。
- `arena_isaac/arena_isaac/run_isaacsim.py`：在完成渲染帧后更新 clearing source。
- `arena_isaac/package.xml`：增加 `sensor_msgs` 和 `sensor_msgs_py` 依赖。
- `arena_isaac/test/test_clearing_geometry.py`：无效值、距离边界和相机方向测试。

### `src/arena/simulation-setup`

- `entities/robots/jackal/model_params.yaml`：Jackal local source 增加 `depth_clearing`。
- `configs/nav2/model_params.yaml`：新增 local source 的安全默认值。
- `configs/nav2/nav2.yaml`：local VoxelLayer 合并额外 observation source。

### 开发根目录

- `scripts/env.sh`：默认启用 `ARENA_DEPTH_CLEARING=true`。
- `scripts/run_six_behaviors.sh`：把开关写入运行 manifest 并打印。
- `scripts/build_costmap_fix.sh`：独立构建和单元测试入口。
- `tools/test_depth_clearing_scene.py`、`tools/costmap_harness/`：真实 Humble VoxelLayer 对照夹具。
- `tools/audit_*.py`：完整场景话题和 Master Costmap 审计。

## Git 记录与隔离

当前已验证版本另以 `baseline/depth-clearing-v1` 标签冻结在根仓库及全部 11 个源码仓库。提交、运行文件校验值和离线 Git bundle 清单记录在 `baseline/depth-clearing-v1.json`；`scripts/verify_depth_clearing_baseline.py` 用于检查标签、稳定源码、安装文件、既存工作区差异和恢复包。规范 LaserScan 在独立 `.workspaces/laserscan-v1` 工作树开发，不修改该稳定源码目录。

开发目录：`/home/lpc/workspace/arena5_ws_costmap_fix`。根目录当前为 `feat/normalized-laserscan-v1`；稳定源码目录保持在清单记录的 `depth_clearing` 提交，规范雷达源码仅位于独立工作树的同名功能分支。

| 仓库 | 基线 | 修复/验证提交 |
| --- | --- | --- |
| 开发根目录 | `89cbc7e` | `583ae36`, `463875a`, `c542faf`, `2ab1b0e` |
| `src/arena-isaac` | `c5a1d8e9561b869c7c67898007948460e6d892ec` | `3d86b950641623ef23ba952d7bd3377d25846816`, `be8fefce4fdeb238372da923214e478d92bbbc32` |
| `src/arena/simulation-setup` | `06574f395de6807f9df842c7369a228902c87256` | `388e5464e958547a50035cc29de21c7da66e9d9a` |

原工作区的恢复期边界检查曾通过：11,438 个文件、11 个 Git 仓库均匹配记录。之后另一个进程在原工作区添加了 `scripts/run_six_behaviors_mpc.sh`；最终只读检查因此报告这一个新增文件，但 11 个仓库的 branch、HEAD、status、staged/unstaged diff 仍全部匹配。本任务没有创建或修改该文件，记录见 `audit/original_verification_final.json`。

## 构建、启动与回退

```bash
cd ~/workspace/arena5_ws_costmap_fix
bash scripts/build_costmap_fix.sh
GPU_ID=3 LIVESTREAM=false bash scripts/run_six_behaviors.sh foxglove:=false
```

开关默认开启。无需改代码的即时回退：

```bash
cd ~/workspace/arena5_ws_costmap_fix
ARENA_DEPTH_CLEARING=false GPU_ID=3 LIVESTREAM=false \
  bash scripts/run_six_behaviors.sh foxglove:=false
```

在独立开发仓库中生成正式反向提交：

```bash
git -C ~/workspace/arena5_ws_costmap_fix/src/arena-isaac \
  revert --no-edit be8fefce4fdeb238372da923214e478d92bbbc32 \
                   3d86b950641623ef23ba952d7bd3377d25846816
git -C ~/workspace/arena5_ws_costmap_fix/src/arena/simulation-setup \
  revert --no-edit 388e5464e958547a50035cc29de21c7da66e9d9a
cd ~/workspace/arena5_ws_costmap_fix
bash scripts/build_costmap_fix.sh
```

查看完整差异：

```bash
git -C ~/workspace/arena5_ws_costmap_fix diff 89cbc7e..HEAD
git -C ~/workspace/arena5_ws_costmap_fix/src/arena-isaac \
  diff c5a1d8e9561b869c7c67898007948460e6d892ec..HEAD
git -C ~/workspace/arena5_ws_costmap_fix/src/arena/simulation-setup \
  diff 06574f395de6807f9df842c7369a228902c87256..HEAD
```

## 官方依据

- [Isaac Sim 5.1 FlatScan 初始化及赋值](https://github.com/isaac-sim/IsaacSim/blob/47d886f2858d1ceed556b21c88927aa67bc81c12/source/extensions/isaacsim.sensors.rtx/nodes/OgnIsaacComputeRTXLidarFlatScan.cpp#L259)
- [Isaac Sim 5.1 ScanBuffer 有效元素筛选](https://github.com/isaac-sim/IsaacSim/blob/47d886f2858d1ceed556b21c88927aa67bc81c12/source/extensions/isaacsim.sensors.rtx/nodes/OgnIsaacCreateRTXLidarScanBuffer.cpp#L1062)
- [Isaac Sim 5.1 CUDA `ElementFlags::VALID` 判断](https://github.com/isaac-sim/IsaacSim/blob/47d886f2858d1ceed556b21c88927aa67bc81c12/source/extensions/isaacsim.sensors.rtx/nodes/IsaacSimSensorsRTXCuda.cu#L55)
- [Isaac Sim ROS 2 LaserScan publisher 原样复制 ranges](https://github.com/isaac-sim/IsaacSim/blob/47d886f2858d1ceed556b21c88927aa67bc81c12/source/extensions/isaacsim.ros2.bridge/nodes/OgnROS2PublishLaserScan.cpp#L148)
- [Isaac Sim 5.1 Generic Model Output](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/docs/source/generic_model_output/generic_model_output.html)
- [Isaac Sim 5.1 Compute RTX Lidar Flat Scan](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.sensors.rtx/docs/ogn/OgnIsaacComputeRTXLidarFlatScan.html)
- [Nav2 Obstacle Layer 参数与 `inf_is_valid`](https://docs.nav2.org/configuration/packages/costmap-plugins/obstacle.html)
- [Nav2 Humble 1.1.18 ObstacleLayer clearing 源码](https://github.com/ros-navigation/navigation2/blob/1.1.18/nav2_costmap_2d/plugins/obstacle_layer.cpp)
- [Nav2 Humble 1.1.18 VoxelLayer clearing 源码](https://github.com/ros-navigation/navigation2/blob/1.1.18/nav2_costmap_2d/plugins/voxel_layer.cpp)

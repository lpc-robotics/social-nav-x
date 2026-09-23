# Arena-Rosnav / Isaac Sim 5.1 激光雷达数据异常与当前修复

## 1. 文档范围

本文记录 Arena-Rosnav 5.0、NVIDIA Isaac Sim 5.1、ROS 2 Humble 和 Nav2 组合中已经复现的激光雷达异常，以及当前开发分支采用的修复方法。

相关目录和分支：

```text
只读原工作区：/home/lpc/workspace/arena5_ws
独立开发区：  /home/lpc/workspace/arena5_ws_costmap_fix
开发分支：    fix/costmap-clearing
```

当前结论只针对已经运行和测量过的 Isaac Sim 5.1 环境。Isaac Sim 6.0 官方声称修复了 RTX LiDAR 的部分不完整扫描问题，但本工作区尚未迁移到 6.0，因此不能把 6.0 的行为视为已经验证。

## 2. 问题摘要

仿真中的动态行人离开后，`local_costmap` 中旧位置的致命障碍值 `254` 可能长时间保留，随后被 InflationLayer 扩张成明显拖影。该拖影可能占用通道、触发局部规划器减速或停止，并造成 DWA/DWB freezing。

最初容易怀疑以下因素：

- LiDAR 实际发布频率不足；
- `observation_persistence` 保存了旧点；
- RViz 显示没有及时刷新；
- InflationLayer 自己保存了历史障碍；
- Nav2 的 `clearing` 参数没有打开。

这些因素已经通过话题时间戳、Master Costmap 数据和运行参数排除。真正问题是：

> 行人离开旧位置后，原始 `/lidar` 经常没有为该方向提供 Nav2 可以使用的 free-space clearing ray。

该问题发生在传感器输出和消费者接口的边界，因此并不只影响 Nav2。Nav2 costmap 拖影只是它在一个有状态占据地图中的具体表现。

## 3. 已确认的运行事实

### 3.1 传感器与 Nav2 配置

```text
/lidar message type:       sensor_msgs/msg/LaserScan
消息时间戳频率：            约 10 Hz（仿真时间）
仿真实时因子 RTF：          约 0.349
Nav2 use_sim_time：         true
local costmap layer：       VoxelLayer
global costmap layer：      ObstacleLayer 配置块存在
observation_persistence：   0.0 s
clearing：                  true
marking：                   true
raytrace_max_range：        3.0 m
obstacle_max_range：        2.5 m
```

`/lidar` 按消息时间戳约为 10 Hz。RTF 只会让 10 秒仿真时间消耗更多墙钟时间，并不表示 Nav2 按仿真时间只收到约 3.49 Hz。因此没有通过提高雷达频率掩盖问题。

### 3.2 原始范围值异常

采样结果中约 46.8% 的 beam 为 `-1` 或 `0`，没有正无穷 `+Inf`。

在真实行人旧位置附近选取连续 20 帧、每帧相邻 7 束，共 140 个样本：

```text
-1：                   91 个
有效有限距离：          49 个
有效距离范围：          4.43–6.66 m
```

这说明同一个关键方向在后续扫描中大量缺少有效 range，而不是每一帧都存在穿过旧行人位置的量程射线。

### 3.3 残留存在于 Master Costmap

直接读取 Nav2 发布的 Master Costmap 后确认：

- 行人旧位置仍有代价值 `254`；
- 周围仍有 inflation 代价值；
- 残留不只是 RViz 显示缓存；
- 降低 RViz decay 或重新打开显示不会清除 Master Costmap。

## 4. 上游数据语义

### 4.1 FlatScan 中 `-1` 的实际来源

Isaac Sim 5.1 的 `OgnIsaacComputeRTXLidarFlatScan` 会先把每个 range bin 初始化为 `-1`，再用通过筛选的 Generic Model Output（GMO）元素覆盖对应 bin。

因此在当前实现中：

```text
LaserScan range == -1
```

只能证明：

```text
这个 FlatScan bin 没有被有效 GMO 元素填充
```

它不能证明：

```text
该方向正常发射，并且在量程内明确没有命中物体
```

ROS2 LaserScan publisher 随后直接复制 FlatScan ranges，没有把 `-1` 标准化成 `+Inf` 或 `range_max`。

对应 NVIDIA 源码：

- [OgnIsaacComputeRTXLidarFlatScan.cpp](https://github.com/isaac-sim/IsaacSim/blob/47d886f2858d1ceed556b21c88927aa67bc81c12/source/extensions/isaacsim.sensors.rtx/nodes/OgnIsaacComputeRTXLidarFlatScan.cpp#L259)
- [OgnIsaacCreateRTXLidarScanBuffer.cpp](https://github.com/isaac-sim/IsaacSim/blob/47d886f2858d1ceed556b21c88927aa67bc81c12/source/extensions/isaacsim.sensors.rtx/nodes/OgnIsaacCreateRTXLidarScanBuffer.cpp#L1062)
- [OgnROS2PublishLaserScan.cpp](https://github.com/isaac-sim/IsaacSim/blob/47d886f2858d1ceed556b21c88927aa67bc81c12/source/extensions/isaacsim.ros2.bridge/nodes/OgnROS2PublishLaserScan.cpp#L148)

### 4.2 `0` 和无效 GMO 数据存在歧义

受控实验打开 `omni:sensor:Core:skipDroppingInvalidPoints=true` 后读取原始 GMO，发现以下几种情况都可能表现为 `distance=0, flags=0`：

- 量程内的开放方向；
- 表面在 far range 以外；
- 遮挡物位于 near range 以内；
- 没有形成有效输出的射线。

所以 `0` 不能作为唯一的 no-hit 标记。`-1`、`0`、NaN 也不能无条件改成 `+Inf`。

### 4.3 与 ROS LaserScan/Nav2 的语义冲突

ROS `sensor_msgs/LaserScan` 约定消费者丢弃小于 `range_min` 或大于 `range_max` 的值。Nav2 Humble 中：

- 普通 LaserScan callback 会在投影时忽略非法范围；
- `inf_is_valid=true` 只处理正无穷；
- 正无穷会被改成略小于 `range_max` 的有限值，再生成 ray；
- `-1`、`0` 和 NaN 不会因为 `inf_is_valid=true` 自动变成 clearing ray。

官方参考：

- [ROS LaserScan message](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/LaserScan.msg)
- [Nav2 Humble ObstacleLayer](https://api.nav2.org/nav2-humble/html/obstacle__layer_8cpp_source.html)
- [Nav2 Obstacle Layer 参数](https://docs.nav2.org/configuration/packages/costmap-plugins/obstacle.html)

## 5. 为什么会形成 costmap 拖影

完整过程如下：

```text
1. 行人在某位置产生有效有限距离
2. VoxelLayer/ObstacleLayer 将对应位置标记为致命障碍 254
3. InflationLayer 在致命单元周围生成膨胀代价
4. 行人离开
5. 后续扫描对应方向是 -1、0 或没有对应点
6. LaserProjection/Nav2 丢弃该 beam
7. Nav2 没有获得穿过旧位置的 clearing ray
8. 旧致命单元继续保留
9. InflationLayer 根据仍存在的致命单元继续生成拖影
```

`observation_persistence=0` 只表示 observation buffer 不额外保存历史消息。它不会在计时结束后自动删除已经写入 costmap 的障碍单元。障碍必须被新的 clearing ray、地图重置或其他明确清除操作移除。

因此关键问题不是“旧观测保存了多久”，而是“新观测有没有证明旧位置现在为空”。

## 6. 已排除或拒绝的方案

### 6.1 优先修改 LiDAR 频率

没有采用。`/lidar` 的消息时间戳已经约为 10 Hz，提高频率不保证缺失角度产生有效 free-space ray，也可能只提高重复无效数据的速率。

### 6.2 调低 `observation_persistence`

没有作用。当前值已经为 `0.0`，且该参数不负责对 Master Costmap 中已标记的单元做时间老化。

### 6.3 仅设置 `inf_is_valid=true`

没有作用。原始 `/lidar` 没有 `+Inf`，Nav2 不会把 `-1/0` 当成正无穷。

### 6.4 把所有 `-1/0/NaN` 改成 `+Inf`

明确拒绝。原始 GMO 实验证明这些数值不能唯一表示“明确无命中”。错误转换可能生成穿过真实墙体、近距离遮挡物或未观测区域的清除射线，造成危险的假清除。

### 6.5 使用 `skipDroppingInvalidPoints=true`

没有作为修复。NVIDIA 曾用它绕过 Isaac Sim 5.0 headless RTX LiDAR 空输出问题，但它只是保留无效条目，并没有赋予这些条目可靠的 free-space 语义。

### 6.6 调整 InflationLayer

没有采用。InflationLayer 只是放大已存在的致命障碍；修改 inflation radius 只能让现象变小，不能清除上游遗留的 `254`。

## 7. 当前修复设计

### 7.1 总体数据流

当前方案不改写 `/lidar`，而是增加一个只负责清除的独立观测源：

```text
Isaac RTX LiDAR
    └── /lidar (LaserScan)
          ├── marking=true
          └── 保留原 clearing 行为

四个同位深度视图
    └── /lidar_clearing (PointCloud2)
          ├── marking=false
          └── clearing=true
```

这样保留 RTX LiDAR 的真实命中作为 marking 证据，同时只使用含义更明确的渲染深度生成 free-space clearing endpoints。

### 7.2 深度视图布置

实现会在 LiDAR prim 下创建四个同位相机：

```text
朝向：0°、90°、180°、270°
每个水平视场：90°
分辨率：321 × 3
使用数据：中间一行
总 endpoint 数上限：4 × 321 = 1284
发布频率：10 Hz 仿真时间
```

四个视图覆盖 360°。原 Jackal LaserScan 配置为约 270°，因此辅助源提供的是围绕机器人完整的渲染 free-space 证据。它只能用于 clearing，不能用于模拟原始 LiDAR marking 或作为未经说明的训练输入。

### 7.3 endpoint 生成规则

对每个深度像素采用保守规则：

```text
有限且大于 range_min + 0.05 m：
    endpoint = measured_depth - 0.05 m

明确的 +Inf：
    endpoint = range_max

0、负数、NaN、-Inf：
    不生成 endpoint
```

有限深度 endpoint 在实际表面前停止 0.05 m，避免 raytrace 穿过被测墙体或人物表面。只有渲染器明确返回的正无穷才延伸到最大量程。

核心代码：

- [`clearing_geometry.py`](src/arena-isaac/arena_isaac/isaac_utils/clearing_geometry.py)
- [`depth_clearing.py`](src/arena-isaac/arena_isaac/isaac_utils/graphs/sensors/depth_clearing.py)
- [`lidar.py`](src/arena-isaac/arena_isaac/isaac_utils/graphs/sensors/lidar.py)
- [`run_isaacsim.py`](src/arena-isaac/arena_isaac/arena_isaac/run_isaacsim.py)

### 7.4 Nav2 observation source

Jackal 当前 local VoxelLayer 合并两个 source：

```yaml
local_observation_sources_string: lidar depth_clearing

depth_clearing:
  topic: ${namespace}/lidar_clearing
  data_type: PointCloud2
  marking: false
  clearing: true
  min_obstacle_height: 0.0
  max_obstacle_height: 2.0
  raytrace_min_range: 0.0
  raytrace_max_range: 3.0
  observation_persistence: 0.0
```

配置位置：

- [`entities/robots/jackal/model_params.yaml`](src/arena/simulation-setup/entities/robots/jackal/model_params.yaml)
- [`configs/nav2/model_params.yaml`](src/arena/simulation-setup/configs/nav2/model_params.yaml)
- [`configs/nav2/nav2.yaml`](src/arena/simulation-setup/configs/nav2/nav2.yaml)

`marking=false` 是安全边界。辅助 endpoint 只表示“从传感器原点到 endpoint 之间可以清除”，不能被当作障碍点重新写入 costmap。

当前 checkout 的 global costmap 中虽然存在 `obstacle_layer` 配置块，但 `plugins` 列表没有加载它，所以 global costmap 当前不消费动态观测。如果以后重新启用 global `obstacle_layer`，应同时确认它是否需要 `depth_clearing`，并重复静态墙和遮挡保护测试。

### 7.5 开关与回退边界

附加源由环境变量控制：

```bash
ARENA_DEPTH_CLEARING=true   # 默认启用
ARENA_DEPTH_CLEARING=false  # 不改代码的即时回退
```

关闭后恢复原始 RTX LaserScan 行为，不创建额外相机和 `/lidar_clearing` publisher。

## 8. 验证结果

### 8.1 受控 Isaac 渲染与真实 Nav2 Humble Master Costmap

基线组和修复组接收相同的缺失回波 LaserScan；修复组额外接收 clearing-only PointCloud2。机器人和传感器固定，移动障碍离开，场景中同时放置静态墙和近距离遮挡物。

| 最终旧区域指标 | 基线 | 修复组 |
| --- | ---: | ---: |
| 致命单元 | 8 | 0 |
| 最大代价 | 254 | 0 |
| inflation 单元 | 156 | 0 |
| 静态墙致命单元 | 20 | 20 |
| 近物体后方记忆单元 | 2 | 2 |

修复组在人离开后第一个 0.5 秒仿真时间采样点已经清空；基线在 5.83 秒后仍有残留。

静态墙保持 `20/20` 个致命单元，近距离遮挡后的记忆单元保持 `2/2`，说明当前测试没有发现穿透静态墙或遮挡物的误清。

验证产物：

- [`audit/depth_clearing_comparison.json`](audit/depth_clearing_comparison.json)
- [`audit/costmap_before_after.png`](audit/costmap_before_after.png)

### 8.2 完整 Jackal、HuNav 和 Nav2 场景

机器人固定在 `(3, 3)`，测试期间位移为 0：

| 场景 | 行人离开后观测时间 | 旧位置致命单元 | 最大代价 |
| --- | ---: | ---: | ---: |
| 原始 LaserScan | 7.50 s | 2 | 254 |
| 增加 clearing source | 2.00 s | 0 | 0 |

`/lidar_clearing` 实测：

```text
频率：           10.0 Hz（消息时间戳）
每帧有限点：     1284
frame_id：       lidar_link
```

完整场景证据：

- [`audit/real_pedestrian_comparison.json`](audit/real_pedestrian_comparison.json)
- [`audit/real_pedestrian_before_after.png`](audit/real_pedestrian_before_after.png)
- [`audit/real_old_position_lidar.json`](audit/real_old_position_lidar.json)

### 8.3 尚未证明的内容

DWB freezing 次数没有在固定机器人试验中直接统计。试验选择固定机器人，是为了把 costmap 清除效果与底盘运动分离。已经证明导致 freezing 的旧位置致命单元从 `254` 清为 `0`，但任务级改善仍应通过运行完整导航任务统计：

- controller 无速度输出持续时间；
- recovery 行为次数；
- 因局部无解产生的取消或超时；
- 多行人通道中的任务成功率和通行时间。

### 8.4 性能代价

四个额外 render products 会增加 GPU 渲染负载。非并发完整场景粗测：

```text
基线 compute：约 13.9–14.7 Hz
修复 compute：约 8.8–9.5 Hz
display：      两组均约 4.9 Hz
```

这不是严格 benchmark，但说明该方法以额外渲染开销换取可靠 clearing。曾尝试将四个相机减少为三个，测试未产生有效 clearing 点，因此已经停止该方案并恢复到验证通过的四相机实现。

## 9. 对后续自研社交导航算法的影响

当前修复没有改变原始 `/lidar`。任何直接订阅 `/lidar` 的算法仍会看到 `-1/0` 和可能的角度缺失。

现有 Arena RL collector 只把 NaN 替换为 `range_max`，保留 `-1/0`：

- [`base_collector_unit.py`](src/arena-rosnav/utils/rl_utils/rl_utils/utils/observation_collector/observation_units/base_collector_unit.py)

现有奖励代码又直接执行 `laser_scan.min()`：

- [`rewards/utils.py`](src/arena-rosnav/utils/rl_utils/rl_utils/utils/rewards/utils.py)

只要扫描中存在 `-1`，最近距离就可能变成 `-1`，从而错误触发安全距离或碰撞惩罚。旧 C++ observation packer 也只处理 Inf 和 NaN，不处理 `-1/0`：

- [`observation_packer.cpp`](src/arena-rosnav/utils/msgs/observations/observation_packer/observation_packer.cpp)

在多行人、多机器人算法中可能出现：

- 将 `0/-1` 误认为贴近机器人表面的障碍；
- 将缺失 beam 误认为自由空间；
- 行人检测漏检或轨迹跳变；
- TTC、最近距离和个人空间指标错误；
- 时序网络学习到 Isaac 特有的缺失模式；
- 仿真训练到真实部署时出现输入分布偏移；
- 动态局部地图再次产生与 Nav2 相同的历史残留。

后续算法不应直接把原始 `ranges` 当作完整距离数组。建议在算法层定义明确的传感器契约：

```text
/lidar_raw         原始数据，仅用于诊断和记录
/lidar_hits        可以确认的实际命中
/lidar_free_space  可以确认的 free-space rays/endpoints
validity mask      HIT / NO_HIT / INVALID / UNKNOWN
```

训练输入至少应把距离和有效性分成两个通道。未知 beam 应作为未知处理，不能无条件当作零距离，也不能无条件当作最大量程。

## 10. Isaac Sim 官方状态

NVIDIA 社区存在 Isaac Sim 5.1 在 FPS 波动时丢失整片角度扇区的报告。NVIDIA 工程师表示已建立内部工单，并在 2026 年 4 月回复该问题已在 Isaac Sim 6.0 修复：

- [NVIDIA Forum：RTX LiDAR incomplete angular sectors](https://forums.developer.nvidia.com/t/rtx-lidar-helper-publishes-incomplete-point-clouds-missing-points-in-certain-angular-regions/348611)
- [IsaacSim issue #319](https://github.com/isaac-sim/IsaacSim/issues/319)
- [Isaac Sim 6.0 Release Notes](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/overview/release_notes.html)

Isaac Sim 官方文档同时说明，旋转式 LiDAR 的完整扫描会跨多个渲染帧累积。移动物体可能在一个完整点云内产生短时拖尾，这是旋转扫描的时间特性；它与数秒后仍没有 clearing ray 的长期 costmap 残留不同：

- [RTX Sensors troubleshooting](https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_rtx.html)

Isaac Sim 6.0 还要求：

```text
omni:sensor:tickRate == omni:sensor:Core:scanRateBaseHz
omni:sensor:Core:accumulateOutputs = true  # 完整 LaserScan
```

不匹配会静默输出 partial scans：

- [Multi-Tick Rendering](https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_multitick_rendering.html)
- [Sensor Timing and RTX Lidar Migration](https://docs.isaacsim.omniverse.nvidia.com/latest/migration_guides/isaac_sim_6_0/sensor_timing_and_rtx_lidar_asset_migration.html)

升级 6.0 是值得进行的隔离验证，但不能在没有 A/B 数据的情况下假定它已经解决当前所有 `-1/0` 语义问题。

## 11. 构建和启动

### 11.1 构建

```bash
cd /home/lpc/workspace/arena5_ws_costmap_fix
bash scripts/build_costmap_fix.sh
```

### 11.2 启动完整场景并启用 Foxglove

```bash
cd /home/lpc/workspace/arena5_ws_costmap_fix

ARENA_FOXGLOVE_ADDRESS=0.0.0.0 \
ARENA_FOXGLOVE_PORT=8875 \
GPU_ID=3 \
LIVESTREAM=true \
ARENA_DEPTH_CLEARING=true \
bash scripts/run_six_behaviors.sh
```

Foxglove 连接地址：

```text
ws://<仿真主机IP>:8875
```

建议同时显示：

```text
/lidar
/lidar/points
/lidar_clearing
/local_costmap/costmap
/local_costmap/published_footprint
/tf
/clock
```

### 11.3 不改代码的 A/B 回退

关闭修复：

```bash
cd /home/lpc/workspace/arena5_ws_costmap_fix

ARENA_DEPTH_CLEARING=false \
GPU_ID=3 \
LIVESTREAM=false \
bash scripts/run_six_behaviors.sh foxglove:=false
```

重新启用：

```bash
ARENA_DEPTH_CLEARING=true \
GPU_ID=3 \
LIVESTREAM=false \
bash scripts/run_six_behaviors.sh foxglove:=false
```

## 12. Git 提交与正式回退

当前实现分布在两个源码仓库中：

| 仓库 | 提交 | 内容 |
| --- | --- | --- |
| `src/arena-isaac` | `3d86b950641623ef23ba952d7bd3377d25846816` | 增加 clearing-only 渲染深度源 |
| `src/arena-isaac` | `be8fefce4fdeb238372da923214e478d92bbbc32` | 完善日志和 render product 生命周期 |
| `src/arena/simulation-setup` | `388e5464e958547a50035cc29de21c7da66e9d9a` | 将 clearing source 接入 Jackal local VoxelLayer |

正式生成反向提交：

```bash
git -C /home/lpc/workspace/arena5_ws_costmap_fix/src/arena-isaac \
  revert --no-edit \
  be8fefce4fdeb238372da923214e478d92bbbc32 \
  3d86b950641623ef23ba952d7bd3377d25846816

git -C /home/lpc/workspace/arena5_ws_costmap_fix/src/arena/simulation-setup \
  revert --no-edit 388e5464e958547a50035cc29de21c7da66e9d9a

cd /home/lpc/workspace/arena5_ws_costmap_fix
bash scripts/build_costmap_fix.sh
```

## 13. 后续验证清单

在修改 LiDAR 实现、迁移 Isaac Sim 6.0 或接入自研导航算法后，至少重复以下检查：

1. `/lidar` 每帧 beam 数是否稳定；
2. `-1/0/NaN/+Inf` 各自比例；
3. 最大连续缺失角度；
4. 每个 beam 能否区分 hit、明确 no-hit 和 unknown；
5. 行人离开后旧位置 `254` 的清除延迟；
6. 静态墙致命单元是否保持；
7. 近距离遮挡后方是否被错误清除；
8. 多机器人互相离开后是否出现历史占据；
9. 激光最近距离、碰撞奖励和 TTC 是否排除了 invalid beam；
10. 训练和评估是否包含真实 LiDAR rosbag 的输入分布对比；
11. 任务级 freezing、recovery、成功率和社交违规次数；
12. clearing render products 对 RTF、GPU 和显存的影响。

只有在“旧动态障碍正常清除”和“静态/遮挡障碍没有误清”同时满足时，新的 clearing 方案才可以视为有效。

# `/lidar_normalized` 替代 Nav2 `/lidar` 实验

## 结论

在当前 Jackal + HuNav + Nav2/DWB 的局部 VoxelLayer 中，`/lidar_normalized` 可以独立完成障碍标记和旧障碍清除。在同一受控场景里，启用 `inf_is_valid=true` 后，人形障碍移走时旧位置 `max_cost` 与稳定的 `/lidar` + `/lidar_clearing` 方案一样降为 0，静态墙和被近物体遮挡的墙仍保留。真实 HuNav 场景也验证了标记到清除的完整过程。

这是一条隔离实验路径，**不是稳定配置的自动迁移**。稳定 `depth_clearing` 基线和原有 Git 标签保持不变，实验代码、构建和日志均位于 `.workspaces/laserscan-v1`。本实验未修改 NVIDIA Isaac Sim 安装源码或 Conda 包。

## 为什么不能只改话题名

REP-117 的 `+Inf` 表示明确无回波。Nav2 VoxelLayer 默认 `inf_is_valid=false`，会丢弃该射线；在受控实验中，若只换为 `/lidar_normalized` 而不打开这个参数，人移走后旧位置仍有 4 个致命障碍格。启用该参数后，Nav2 将 `+Inf` 投影到量程末端，用它执行 raytrace clearing。为避免末端被当成障碍，实验把 `obstacle_max_range` 保持在 2.5 m、`raytrace_max_range` 保持在 3.0 m，均小于该雷达 12 m 的 `range_max`。

隔离的 [Jackal 参数](.workspaces/laserscan-v1/src/simulation-setup/entities/robots/jackal/model_params.yaml) 将 observation source 换成 `normalized`，设置 `marking=true`、`clearing=true`、`inf_is_valid=true`，不再在局部地图中使用 `/lidar` 或 `/lidar_clearing`。原 `/lidar` 话题仍可发布，但没有局部地图订阅者，方便回退和兼容其他实验。DWB 本身仍消费局部代价地图，而非直接订阅 LaserScan。

## 验收证据

| 真实 VoxelLayer 受控场景 | 人离开前致命格 | 人离开后旧位置最高代价 | 静态墙 | 近物体后被遮挡墙 |
| --- | ---: | ---: | --- | --- |
| 稳定 `/lidar` + `/lidar_clearing` | 8 | 0 | 20→20 | 保留 1 格 |
| `/lidar_normalized`，`inf_is_valid=false` | 7 | 254 | 28→28 | 保留 |
| `/lidar_normalized`，`inf_is_valid=true` | 7 | 0 | 28→28 | 4→4 格 |

近物体产生 95 束 `-Inf`；它们没有清除后方墙。不同雷达的束角和采样密度不同，因此表中的墙体格数不要求逐格相等；验收的是“旧障碍清空、静态与遮挡障碍保留”的效果，而非整张栅格逐字节一致。原始结果保存在 `.workspaces/laserscan-v1/log/normalized_nav2_costmap_comparison.json`。

完整场景中，实际读取到 `FollowPath.plugin=dwb_core::DWBLocalPlanner`、`voxel_layer.observation_sources=normalized`、`voxel_layer.normalized.inf_is_valid=true`；`/lidar_normalized` 的唯一订阅者是 `/local_costmap/local_costmap`，`/lidar` 没有局部地图订阅者。最终单源运行还设置 `depth_clearing=false`，其话题未出现。真实 HuNav 行人先形成 6 个致命格，移动 1.70 m 后旧位置最高代价为 0；机器人没有移动。机器可读摘要见 [验收记录](audit/normalized_nav2_replacement_validation.json)。

## 运行与回退

```bash
cd /home/lpc/workspace/arena5_ws_costmap_fix
bash scripts/build_normalized_nav2_experiment.sh
LIVESTREAM=false GPU_ID=0 bash scripts/run_normalized_nav2_experiment.sh foxglove:=false
```

实验入口强制 `NAVIGATION=true`、`ARENA_NORMALIZED_SCAN=true`、`ARENA_DEPTH_CLEARING=false`，并检查隔离版 Nav2 参数确实装入 overlay。使用哪张 GPU 可以按本机情况调整 `GPU_ID`。稳定版可立即从新终端启动：

```bash
bash scripts/run_depth_clearing_baseline.sh foxglove:=false
scripts/verify_depth_clearing_baseline.py
```

两套启动入口不要同时运行，以免 ROS 话题、端口和 GPU 资源冲突。根仓库和隔离的 `simulation-setup` 工作树均用 `validated/normalized-nav2-replacement-v1` 注解标签保护；稳定基线标签仍为 `baseline/depth-clearing-v1`。

## 尚未覆盖的安全边界

- Nav2 的 LaserScan 投影不会把 REP-117 `-Inf` 变为近距障碍标记；虽然它不会误清遮挡区，但不能据此宣称近距碰撞安全已完成。部署前需要独立的近距停障/碰撞监测措施。
- `NaN` 表示未知，不可当作自由空间；当前 Nav2 会忽略这类束。
- 本实验没有证明多机器人实时性能。此前 1/2/4 路规范雷达的短时实时因子约为 0.86/0.54/0.37，不能直接作为实时多机器人训练输入。
- 完整场景使用 `amcl=false`。虽然隔离参数的首选雷达源也指向规范雷达，启用 AMCL 的定位行为仍需单独验证。

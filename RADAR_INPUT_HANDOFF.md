# 雷达输入方案交接：已验收版本

本页供后续 Codex agent、DRL/MPC/多机器人算法和 Nav2 开发者快速定位。2026-09-16 用户已验收当前 `/lidar_normalized` 单源 Nav2 方案；此前的 `/lidar_clearing` 仍是独立、冻结且可随时启动的稳定基线。**“已验收”不等于修改稳定默认启动配置**：两套入口继续隔离，均不得修改 NVIDIA Isaac Sim 安装源码或 Conda 环境。

| 用途 | 话题与类型 | 数据语义/代价地图职责 | 代码与启动入口 |
| --- | --- | --- | --- |
| RTX 兼容输出 | `/lidar`，`LaserScan` | 原值可能有语义不明的 `-1`、`0`；不可把它们视作确定的 free-space | 稳定 `src/arena-isaac`；与下行一起用于稳定 Nav2 |
| 稳定清除基线 | `/lidar_clearing`，`PointCloud2` | 独立渲染深度生成 clearing ray；局部 VoxelLayer **仅 clearing，不 marking**，原 `/lidar` 负责 marking | 稳定 `src/`；`scripts/run_depth_clearing_baseline.sh` |
| 统一算法输入 / 已验收 Nav2 替代 | `/lidar_normalized`，`LaserScan` | 独立渲染深度重建 REP-117 语义；隔离配置中是局部 VoxelLayer **唯一** marking + clearing source | `.workspaces/laserscan-v1/src/`；`scripts/run_normalized_nav2_experiment.sh` |

## 规范雷达数据契约及消费

当前 Jackal 为 640 束、360°、0.08–12 m、仿真时间 10 Hz；实际参数由加载的 URDF 读取。`lidar_link_normalized` 有相对 `lidar_link` 的静态 TF，当前 z 偏移 0.142 m。有限且在量程内的值是命中；`+Inf` 是确认无回波；`-Inf` 是确认过近，不能用于 clearing；`NaN` 是未知，不能用于 clearing 或推断空闲。规范话题不使用有限 `-1`、`0` 哨兵。详见 [数据契约](NORMALIZED_LASERSCAN_CONTRACT.md) 与 [几何/生命周期验证](NORMALIZED_LASERSCAN_VALIDATION.md)。

DRL/MPC 消费端不要直接把 `NaN`、`-Inf` 改成最大量程。使用 `isaac_utils.scan_geometry.adapt_scan_for_control` 的有限数组和 `hit`、`no_return`、`too_close`、`unknown`、`usable_ray`、`obstacle` 掩码，并在训练/控制接口中保留状态语义。多机器人订阅各自命名空间（如 `/robot_1/lidar_normalized`），不能混用 TF、时间状态或缓存。

## Nav2/DWB 连接方式与验收范围

稳定配置在 `src/arena/simulation-setup/entities/robots/jackal/model_params.yaml`：`/lidar` 标记/清除，`/lidar_clearing` 仅清除。隔离的已验收配置在 `.workspaces/laserscan-v1/src/simulation-setup/entities/robots/jackal/model_params.yaml`：`observation_sources=normalized`，话题为 `${namespace}/lidar_normalized`，`marking=true`、`clearing=true`、`inf_is_valid=true`，不再订阅 `/lidar` 或 `/lidar_clearing` 作为局部 VoxelLayer 源。DWB (`dwb_core::DWBLocalPlanner`) 消费局部代价地图，**不直接消费点云或 LaserScan**。

`inf_is_valid=true` 是必要条件：不启用时受控场景的旧行人位置仍有 4 个致命格；启用后最高代价降为 0，与稳定基线的清除结果一致。完整 Jackal + HuNav + Nav2 场景中行人先被标记为 6 个致命格，移动约 1.70 m 后旧位置最高代价为 0，机器人保持静止；静态墙和被近物体遮挡的墙未被误清。细节和机读结果见 [Nav2 替代报告](NORMALIZED_NAV2_REPLACEMENT_REPORT.md) 与 [验收摘要](audit/normalized_nav2_replacement_validation.json)。

此验收证明局部 VoxelLayer 单源 marking/clearing，不等于证明整张 costmap 逐格相同、DWB 行驶任务无 freezing、AMCL 定位已验证或多机器人实时性能达标。Nav2 对 `-Inf` 近距状态不会生成障碍 marking；真实部署仍需独立近距停障/碰撞保护。

## 构建、启用、核验与回退

从仓库根目录，**不要同时运行两套仿真**。先构建隔离 overlay，再启动已验收方案：

```bash
bash scripts/build_normalized_nav2_experiment.sh
LIVESTREAM=false GPU_ID=0 bash scripts/run_normalized_nav2_experiment.sh foxglove:=false
```

此入口强制 `NAVIGATION=true`、`ARENA_NORMALIZED_SCAN=true`、`ARENA_DEPTH_CLEARING=false`，并检查隔离版 Jackal 参数与已安装 overlay 一致。`GPU_ID` 可按机器调整。稳定启动入口独立地强制 `ARENA_DEPTH_CLEARING=true`、`ARENA_NORMALIZED_SCAN=false`：

```bash
scripts/verify_depth_clearing_baseline.py
LIVESTREAM=false GPU_ID=0 bash scripts/run_depth_clearing_baseline.sh foxglove:=false
```

若只是关闭新源，结束实验进程并从新终端使用稳定入口即可；不要复制隔离工作树的参数到稳定 `src/`。若需恢复已验收源码，先读 `baseline/radar-input-accepted-v1.json` 中的精确提交与标签，在干净的新工作树检出，不要对含用户修改的工作树运行 `reset --hard`/强制 checkout。离线增量 bundle 依赖 `baseline/depth-clearing-v1` 的完整 bundle；两级保护分别保存在 `logs/baselines/depth-clearing-v1/bundles/` 与 `logs/baselines/radar-input-accepted-v1/bundles/`。

验收版本完整校验命令为 `python scripts/verify_radar_input_accepted.py`；仅检查运行所需文件而跳过离线 bundle 哈希时使用 `--runtime-only`。它同时调用稳定基线校验，防止保存新方案时破坏旧方案。

## 版本边界

- 稳定标签 `baseline/depth-clearing-v1`，清单 [baseline/depth-clearing-v1.json](baseline/depth-clearing-v1.json)。其源码、已安装运行文件、Isaac Sim/Conda 环境保护及离线恢复包由 `scripts/verify_depth_clearing_baseline.py` 校验。
- 此次用户验收标签 `accepted/radar-input-v1`：根仓库文档/入口，以及隔离 `arena-isaac` 和隔离 `simulation-setup` 的源码提交；精确对象与恢复包校验值见 [验收清单](baseline/radar-input-accepted-v1.json)。历史 `validated/normalized-laserscan-v1.1` 和 `validated/normalized-nav2-replacement-v1` 不移动。
- 根目录 `scripts/env.sh` 的通用默认值仍为 `ARENA_DEPTH_CLEARING=true`、`ARENA_NORMALIZED_SCAN=false`。不要把隔离方案误当成通用默认值。

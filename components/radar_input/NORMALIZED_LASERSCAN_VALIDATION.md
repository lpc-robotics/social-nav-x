# 规范化 LaserScan 验收报告

## 结论

规范雷达 v1.1 已经通过语义、几何、时间、TF、动态场景、生命周期、多机器人隔离和稳定 costmap 回归验收。新功能默认关闭，现有 `/lidar` 与 `/lidar_clearing` 不变；当前 Nav2 继续只消费稳定的 clearing 源。v1.1 在不改变接口的前提下补齐 URDF 高斯噪声均值、render 读取失败时“不发布并报错”的行为，以及控制适配器有限值约束。

实现没有修改 NVIDIA Isaac Sim 5.1 源码或 Conda 包。源码位于独立 `.workspaces/laserscan-v1/src/arena-isaac` 工作树，构建、安装和日志分别隔离在该目录的 `build`、`install`、`log`。`vendor/IsaacSim` 保持提交 `47d886f2858d1ceed556b21c88927aa67bc81c12` 且工作树洁净，Isaac 环境的 `conda-meta/history` 校验值未变化。

机器可读结果见 `audit/normalized_laserscan_validation.json`。
验证版本入口记录在 `baseline/normalized-laserscan-v1.json`，根仓库和独立 `arena-isaac` 工作树当前均使用 `validated/normalized-laserscan-v1.1` 标签保护。原 `validated/normalized-laserscan-v1` 标签保持原指向，未移动或覆盖，可作为上一版本回退点。

## 接口与边界

```text
/lidar              原 RTX LaserScan；保持不变
/lidar_clearing     已验证的 clearing-only PointCloud2；保持启用
/lidar_normalized   新 REP-117 LaserScan；实验开关默认关闭
```

新话题只发布四类明确状态：量程内有限命中、`+Inf` 无回波、`-Inf` 确认过近和 `NaN` 未知。不会把 RTX FlatScan 的 `-1` 或 `0` 猜测成空闲空间。面向控制器的 `adapt_scan_for_control` 另行生成有限数组以及 `hit/no_return/too_close/unknown/usable_ray/obstacle` 掩码，未知默认保守填 0。

第一版明确不包含反射强度、材质回波、多回波和逐束运动畸变，也不把现有 Nav2、DRL 或 MPC 直接迁移到新话题。

## 关键验收

| 项目 | 结果 |
| --- | --- |
| 独立 overlay 构建 | `arena_isaac`、`arena_humble_compat` 成功 |
| stable/离线恢复校验 | runtime 49 项、含 bundle 61 项全部通过 |
| 几何/语义/噪声确定性测试 | 12/12 通过 |
| 新文件 ament flake8 / pep257 | 通过 |
| 受控墙体距离 | 期望 2.9 m，实测 2.9000001 m |
| 动态物体 | 1.8000 m 命中，移走后为 `+Inf` |
| 过近物体 | `-Inf` |
| 完整 Jackal + HuNav | 25 帧、640 束、10 Hz、非法有限值 0 |
| v1.1 实景复验 | 10 帧、640 束、10 Hz、非法有限值 0 |
| 360° 布局 | `2π/640`，不重复首尾束 |
| 时间 | `scan_time=0.1 s`、`time_increment=0`、仿真时钟 |
| 实际 TF | `lidar_link → lidar_link_normalized`，z=0.142 m |
| 同路径重生 | 重建后 15 帧、10 Hz |
| 最终资源状态 | 活跃实例 0，annotator/render product 清理告警 0 |
| stable costmap 回归 | 旧区域 254→0，墙体 20、遮挡后单元 2 均保留 |

完整导航场景中 `/lidar_normalized` 有 1 个 publisher、0 个 subscriber；`/lidar_clearing` 有 1 个 publisher，并仍由 `/local_costmap/local_costmap` 订阅。这验证了新接口与稳定导航链路隔离。

## 多机器人性能

独立规范雷达源在 RTX 4090 上的短时测试如下。每个机器人均保持独立话题、frame、节流状态和确定性随机种子，仿真频率均为 10 Hz。

| 规范雷达源 | 实时因子 |
| ---: | ---: |
| 1 | 0.86 |
| 2 | 0.54 |
| 4 | 0.37 |

因此 1/2/4 路的功能隔离已经验证，但当前每个雷达使用四个独立 1025×3 render products；4 路不满足实时训练性能目标。后续如需实时多机器人训练，应单独评估 tiled rendering、GPU 侧采样与批量发布，不能把本轮功能验收结果解释为实时性能达标。

## 测试基础设施说明

定向构建、显式 pytest 和 ament lint 均通过。仓库的通用 `colcon test` 会由 setuptools 在普通 ROS Python 中自动导入整个 `arena_isaac.services`/`isaac_utils.graphs` 包；这些包要求先由 Kit 加载 `omni` 扩展，因此在非 Isaac Kit 进程中报 `ModuleNotFoundError: omni`。这是现有测试发现方式的限制，错误还解析到了稳定源码目录，不是规范雷达几何或运行测试失败；本轮没有扩大范围去重构全包测试入口。

## 运行与回退

```bash
cd /home/lpc/workspace/arena5_ws_costmap_fix
bash scripts/build_normalized_scan.sh
GPU_ID=3 LIVESTREAM=false bash scripts/run_normalized_scan_experiment.sh foxglove:=false
```

即时回到经过验证的 stable 路径：

```bash
bash scripts/run_depth_clearing_baseline.sh foxglove:=false
```

该入口先校验稳定源码和安装文件，再强制使用共享安装、`ARENA_DEPTH_CLEARING=true`、`ARENA_NORMALIZED_SCAN=false`。完整标签和离线恢复包校验为：

```bash
scripts/verify_depth_clearing_baseline.py
```

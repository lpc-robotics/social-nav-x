# 规范化 LaserScan 数据契约

## 目的

`/lidar_normalized` 是 Isaac Sim 5.1 中面向 DRL、MPC 和多机器人算法的统一二维雷达接口。它使用独立渲染深度视图重建量程与遮挡关系，不解释 RTX FlatScan 中语义不明的 `-1` 和 `0`。

原有话题继续承担已经验证的职责：

```text
/lidar              RTX LaserScan，保留现状
/lidar_clearing     clearing-only PointCloud2，供 Nav2 稳定基线使用
/lidar_normalized   REP-117 LaserScan，供新算法开发使用
```

当前 Nav2 配置不迁移到 `/lidar_normalized`。关闭新话题不会影响 `depth_clearing`。

## 消息语义

消息类型为 `sensor_msgs/msg/LaserScan`，距离值遵循 REP-117：

| 状态 | `ranges` | 消费规则 |
| --- | --- | --- |
| 有效命中 | `range_min <= r <= range_max` | 障碍端点，原点到端点之间为已观测射线 |
| 量程内无回波 | `+Inf` | 可用的空闲射线，终点为 `range_max` |
| 确认过近 | `-Inf` | 紧邻传感器的障碍告警，不能用于清除 |
| 未知或无效 | `NaN` | 不能推断障碍或空闲空间 |

规范话题禁止发布有限的 `-1`、`0` 或量程外哨兵值。渲染深度中的负数、`NaN` 和 `-Inf` 转为未知；当前安装版 `distance_to_camera` 定义的 `0` 以及实际输出的 `+Inf` 转为无回波。

## 扫描几何和时间

- 束数、视场、频率和量程从实际加载的 URDF 读取。当前 Jackal 为 640 束、360°、0.08–12 m、10 Hz。
- 360° 扫描不重复首尾方向，始终满足 `angle_max = angle_min + (N-1) * angle_increment`。
- 第一版为同一渲染帧的同步扫描，所以 `time_increment=0`，`scan_time=1/update_rate`。
- `header.stamp` 使用仿真时间；仿真暂停时不发布，时间回退时重置节流状态。
- `header.frame_id` 为 `<原 frame>_normalized`。静态 TF 描述 URDF 中的真实传感器位姿，当前包含相对 `lidar_link` 的 0.142 m 高度偏移。
- `intensities` 为空。第一版不模拟反射强度、多回波或旋转雷达的逐束运动畸变。

## 算法适配

公共函数 `isaac_utils.scan_geometry.adapt_scan_for_control` 返回有限数组及以下布尔掩码：

- `hit`：有效障碍命中；
- `no_return`：明确无回波；
- `too_close`：确认过近；
- `unknown`：无可靠信息；
- `usable_ray`：可用于几何射线的 `hit | no_return`；
- `obstacle`：需要作为障碍处理的 `hit | too_close`。

有限数组中，无回波填充为 `range_max`，过近和未知填充为 `0`。DRL 必须同时使用掩码或在模型输入定义中明确保守填充值；不得沿用当前把 `NaN` 直接改成 `range_max` 的处理。MPC 只能用 `usable_ray` 建立可见区域，并把 `obstacle` 用作碰撞约束。

每个机器人订阅自己的命名空间，例如 `/robot_1/lidar_normalized`，并保留独立的时间状态、TF 和随机种子派生值。禁止将多个机器人的扫描合入共享无命名空间缓存。

## 构建、运行和回退

```bash
cd /home/lpc/workspace/arena5_ws_costmap_fix
bash scripts/build_normalized_scan.sh
GPU_ID=3 LIVESTREAM=false bash scripts/run_normalized_scan_experiment.sh foxglove:=false
```

几何验收时关闭噪声：

```bash
ARENA_NORMALIZED_SCAN_NOISE=off GPU_ID=3 LIVESTREAM=false \
  bash scripts/run_normalized_scan_experiment.sh foxglove:=false
```

新功能即时关闭：

```bash
ARENA_NORMALIZED_SCAN=false ARENA_DEPTH_CLEARING=true \
  bash scripts/run_six_behaviors.sh foxglove:=false
```

完整稳定基线入口会先核对源码和安装文件：

```bash
bash scripts/run_depth_clearing_baseline.sh foxglove:=false
```

基线标签、提交和离线 bundle 清单记录在 `baseline/depth-clearing-v1.json`。完整校验命令为：

```bash
scripts/verify_depth_clearing_baseline.py
```

校验同时保护 `vendor/IsaacSim` 的源码提交/洁净状态以及 Isaac Conda 环境的历史文件。实验构建只写入 `.workspaces/laserscan-v1/{build,install,log}`；运行期缓存重定向到工作区 `.cache`，不向 Isaac Sim 安装目录安装或覆盖任何源码与包。

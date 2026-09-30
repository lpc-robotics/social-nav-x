# 行人对双机器人反应的受控对照实验

日期：2026-09-29。范围：纯 CPU 动力学实验，不是 Isaac 全链路验收。没有向现有仿真发布指令。

## 场景与控制变量

世界 30×23 m；一个 Regular 行人，group_id=-1，初始位置 (10,10)，速度零，目标 (16,10)，目标半径 0.3 m，行人半径 0.4 m，期望速度 1 m/s。两台机器人静止，半径均为 0.35 m。积分步长 0.025 s，观察 30 s，进入目标区后冻结轨迹用于作图。机器人始终保留在物理场景和净距评价中，只对模型输入做遮罩。

| 场景 | R1 | R2 | 双机模型结果 |
|---|---|---|---|
| symmetric | (11,9.2) | (11,10.8) | 30 s 内未到达；窄口前停滞 |
| staggered | (11,9.2) | (11.6,10.8) | 30 s 内未到达；错位窄口仍停滞 |
| wide | (11,9.2) | (11.6,11.2) | 6.85 s 进入目标区 |

以上场景按从对称窄口到错位、再到扩大间隙的顺序探索，所有结果都保留，不仅保留成功场景。它们是确定性案例，不代表随机场景的统计成功率。

## 对照组

- legacy_r1 / legacy_r2：使用与原 arena5_ws 完全相同的 LightSFM 头文件内核，复现原 Regular 单行人 computeForces + updatePosition 分支，只接收一个机器人。统一速度、墙边界与步长；这不是启动原 HuNav 行为树服务的端到端实验。
- current_r1 / current_r2：直接编译并调用当前 core.cpp，仅保留指定机器人的输入，保留新版近距离项、限幅等全部机制。用于严格消融。
- current_both：当前核心同时接收两个机器人。
- current_nearest：同一当前核心每步只接收最近机器人，距离相等时选 R1。用于排除“只对最近目标反应”的替代解释。
- current_none：当前核心无机器人输入，作为无交互控制。

原内核和新版除了机器人数量之外，还存在近距离项、加速度限幅、奇异值处理差异。不能将 legacy_r1 与 current_both 的全部差异归因于多机。因果归因应使用 current_r1/current_r2/current_nearest 与 current_both 的对照。

## wide 场景结果

净距 = 人机中心距离 - 行人半径 - 机器人半径；负数表示本实验圆形几何包络重叠，不是 Isaac 物理碰撞实测。

| 模式 | 到 R1 最小净距/m | 到 R2 最小净距/m | 到达时间/s |
|---|---:|---:|---:|
| 原内核只接收 R1 | 0.137 | 0.361 | 5.95 |
| 当前只接收 R1 | 0.413 | -0.135 | 6.30 |
| 当前只接收 R2 | -0.040 | 0.608 | 6.025 |
| 当前只接收最近机器人 | 0.280 | 0.264 | 6.55 |
| 当前同时接收两台 | 0.318 | 0.267 | 6.85 |

当前双机相对当前 R1-only 最大同时间位置差 0.742 m；相对 R2-only 为 0.911 m；相对 nearest-only 为 0.305 m。相对原内核 R1-only 的最大横向差 0.245 m、最大同时间位置差 0.897 m。同时间位置差包含速度/时间进度差，不能全当作横向绕行幅度。

在 wide 的 4.25 s 时间内，同一计算步中 R1 和 R2 贡献的模长都大于 0.01。贡献记录在合加速度限幅前；不能要求限幅后的动作简单满足线性叠加。

## 现有验证

existing_tests.txt：三项既有 CPU 测试全部通过：AllRobotsHaveIndependentContributions、RobotOrderAndNamesDoNotChangeMotion、AblatingEitherRobotChangesTrajectoryByTenCentimeters。它们来自已有 build/test_core；本次轨迹实验则重新编译当前核心源码，源文件哈希见 manifest.json。

## 复现

在 /home/lpc/workspace/arena5_multi_ws 下运行：

```bash
python3 evidence/robot_response_comparison/run.py
MPLCONFIGDIR=/tmp/arena_response_mpl /home/lpc/miniforge3/envs/isaaclab/bin/python evidence/robot_response_comparison/plot.py
```

run.py 从现有构建读取编译与链接参数，将独立可执行文件、源码哈希和 CSV 保存到本目录。plot.py 生成三组 PNG/PDF 和 summary.json。无需启动 ROS 节点，不调用运动服务。

## Isaac 全链路复验设计

使用同样坐标和参数，external 控制模式下两台机器人固定不动，关闭其 Nav2 自主避让，以排除机器人先让路造成的混淆。各组重置同一初态：旧后端固定参考 R1；新版用显式输入遮罩分别保留 R1、R2、最近机器人或两台。遮罩应在独立实验适配器中实现，保持请求与响应校验一致，不能只停止订阅造成时间戳超时。不要把未选机器人删除出世界，或作为障碍/行人重新送回模型。

同步记录 /multirobot/hunav/agents（权威状态）、/multirobot/hunav/robots（实际输入）、/multirobot/hunav/interactions（逐机器人力）、/multirobot/hunav/actual_people（实际显示角色）、/clock，以及两台机器人的 odom。使用仿真时间对齐，同时报告模型轨迹、实际角色轨迹、两机器人最小净距、到达时间及停滞。

动态扩展：将 R1 设为 (11,8.8+0.2t)，R2 设为 (11.6,11.6-0.2t)，0≤t≤4 s，之后停车。各对照组回放完全相同的机器人位姿和速度序列；这是待执行的动态场景，不是本次已验证结果。另对宽间隙场景将行人初始 y 做 ±0.1 m 小扰动和镜像重复，防止结论只依赖单个几何配置。

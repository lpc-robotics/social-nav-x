# 形式化社交自动机 V1：开发基线与保护边界

本文记录 `formal-social-automata-v1` 开发开始前已经核验的事实。后续 Codex
Agent 必须先读本文和仓库根目录的
`FORMAL_SOCIAL_AUTOMATA_DEVELOPMENT_PLAN.md`，再修改代码。本文不是运行结果的
预告；未列为“已验证”的项目必须通过测试后才能写入交付结论。

## 1. 权威来源与文档优先级

开发源码的唯一基线是：

- 归档仓库：`/home/lpc/workspace/social-nav-x-archive`
- canonical commit：`51ab117dedf6a8173c1704f0edd8d01c7938fb8e`
- tag：`arena5-isaac5.1-archive-20260829`
- feature 分支：`feature/formal-social-automata-v1`
- 隔离 worktree：`/home/lpc/workspace/social-nav-x-formal-v1`

开发说明的原始附件已经逐字保存为
`docs/formal_social_automata/source_spec_20260830.md`。它及外部原稿
`/home/lpc/social-nav-x_formal_social_automata_development_spec.md` 的 SHA-256 均为：

```text
e033334817f3a6096845765969c5af57444644d32d6df618b34de16b9f12a268
```

原稿用于追溯研究意图，不是实现接口的最终依据。发生冲突时，优先级如下：

1. 用户确认的 V1 要求；
2. 根目录 `FORMAL_SOCIAL_AUTOMATA_DEVELOPMENT_PLAN.md`；
3. 本文记录的基线事实与保护边界；
4. `source_spec_20260830.md` 中仍然适用的研究背景。

不得为了让实现迎合原稿而改写原稿；对原稿假设的修正写在权威计划中。

## 2. 两层项目结构

项目存在两个用途不同的层次：

- `/home/lpc/workspace/social-nav-x-formal-v1` 是从干净归档 commit 创建的、可提交的
  V1 开发 worktree，也是本次唯一允许修改的源码树。
- `/home/lpc/workspace/arena5_ws` 是已经部署和验证的实际运行工作区。其根目录不是
  Git 仓库，内部由 11 个独立上游仓库组成；多个仓库包含有意保留的未提交修改和
  intent-to-add 项。一个顶层 Git commit 不能代表该工作区状态。

活动工作区只提供已安装 ROS 2/HuNav 依赖和运行回归环境。不得在其中执行
`git reset`、`git clean`、`git checkout`、`git pull` 或 `git rebase`，不得直接编辑
它的源码、配置、Conda 环境、build/install/log 目录。特别禁止运行归档仓库原有的
全量 `scripts/build.sh`，因为它会在活动工作区生成文件并操作嵌套仓库的
intent-to-add 状态。

归档仓库直接保存项目自有源码和五个上游补丁，其他依赖按
`upstream/manifest.tsv` 固定提交重建。不得修改 `.repos`、该 manifest、lockfile 或
依赖版本来完成 V1。

## 3. 恢复保障

完整工作区恢复包位于：

```text
/home/lpc/workspace/arena5_ws_archives/20260829_164103_full_workspace_pre_log_cleanup/arena5_ws_full.tar.zst
```

其 SHA-256 为：

```text
69bc5468d9f3e4674fd61eb637242e3744cf8c741a31e1b243892997af2160e0
```

恢复说明：

```text
/home/lpc/workspace/arena5_ws_archives/20260829_164103_full_workspace_pre_log_cleanup/RESTORE.md
```

关键文件校验清单：

```text
/home/lpc/workspace/arena5_ws_archives/20260829_164103_full_workspace_pre_log_cleanup/KEY_SHA256SUMS
```

常规回退不需要还原快照：停止 formal demo，打开一个未 source feature overlay 的新
shell，继续使用原 `arena5_ws` 即可。只有活动工作区本身损坏时才执行完整恢复；必须
先停止该工作区进程、把现有 `arena5_ws` 改名保留，再按 `RESTORE.md` 解压，禁止覆盖式
解压。Isaac Sim 环境 `/home/lpc/miniforge3/envs/isaaclab` 不在恢复包中，也不属于本次
修改范围。

## 4. 已验证运行基线

归档证据显示下列链路已经通过：

- D6 速度矩阵 22/22；
- 碰撞套件 3/3；
- Nav2 smoke test，期望 marker 为 `SMOKE_NAVIGATION_OK`；
- 六行为 demo，期望 marker 为
  `SIX_BEHAVIORS_VERIFY_OK types=1,2,3,4,5,6`；
- 长时间运行约 `12.6--14.7 Hz` wall-clock HuNav compute 和 `4.9 Hz` Isaac
  Character display。

现有固定行为链路为：

```text
HuNav YAML
  -> hunav_loader
  -> hunav_agent_manager /compute_agents
  -> arena_humble_compat/hunav_six_behaviors_bridge
  -> arena_people_msgs SpawnPedestrians / UpdatePedestrians
  -> arena_isaac services
  -> Isaac Character
```

HuNav 是行为和连续运动 authority；bridge 将 `/odom` 中的机器人 pose/rigid-body
velocity 送给 HuNav；Isaac Character 只显示 HuNav 返回的 pose、yaw、velocity 和动画。
V1 不能把人物 pose 直接写入 Isaac，也不能把自动机逻辑放进 `Person.py`。

原入口 `scripts/run_six_behaviors.sh`、六人 YAML/launch、D6、odom、Nav2 和 Isaac
Character 均属于守护基线。formal demo 必须有独立入口，原入口及其默认行为不得改变。

## 5. 已确认的 HuNav v1 行为切换事实

当前 HuNav v1（固定上游 commit
`a69cf96d98b0d40e247f819d7aebab661ac68b3b` 加归档补丁）已经在
`hunav_agent_manager` 内使用 BehaviorTree.CPP v3。它在第一次 compute 时根据每个
agent 的 `behavior.type` 创建并缓存一棵 XML 行为树；后续 compute 虽会更新消息字段，
但不会因为请求中的 type 改变而重新选择树。

因此，原稿中的“每拍直接改 `agent.behavior.type` 即动态切换”不成立。V1 不修改 HuNav
核心，而是在独立 ROS 服务代理中：当完整 behavior profile 发生变化时调用现有
`hunav_msgs/srv/ResetAgents`，再用当前 agent/robot 状态调用 raw
`hunav_msgs/srv/ComputeAgents`，让 HuNav 从新 profile 重建树。`ResetAgents` 会清除所有
树和内部 agent 状态，下一次 compute 负责初始化；其请求携带的状态不被当前回调直接
使用。

V1 固定一个行人，所以“全体 reset”可接受，但存在一次计算拍的可见切换延迟。扩展到
多人前必须重新评估按 agent 动态切树，不能直接复用这一假设。

## 6. 已确认的感知与场景约束

- 当前 DWB 配置 `max_vel_x` 和 `max_speed_xy` 均为 `0.26 m/s`。因此
  `0.8 m/s` 快速接近不可能由默认 Nav2 产生；该验收场景必须在
  `NAVIGATION=false` 时由单一、受控 `/cmd_vel` 发布者执行。
- V1 的可见性只由二维距离和行人 yaw/FOV 判定，不接入地图遮挡、ray tracing 或
  Isaac 视觉。文档和 topic 不能把它描述为真实视线遮挡判断。
- V1 的个人空间以项目现有中心距 `1.0 m` 作为进入阈值；它不是两个 collider 的边缘
  距离，也不是新的物理碰撞保证。
- 所有几何量在 `map` 平面计算；所有 dwell、timeout、cooldown 只使用 compute 请求的
  ROS simulation stamp，不使用 wall clock。
- 正式 demo 默认 `GPU_ID=3`、`NAVIGATION=false`，避免 Nav2 与测试控制器同时发布
  `/cmd_vel`。

## 7. 构建与依赖保护

使用活动工作区 `/home/lpc/workspace/arena5_ws/install` 和
`/home/lpc/workspace/arena5_ws/.conda/arena_ros` 作为只读 underlay。在 feature worktree
中使用独立 `build/`、`install/`、`log/` 构建 overlay，只选择新增包及确有改动的兼容
包。

禁止执行或引入：

- `sudo`、`apt`、`rosdep`、`pip`、`conda`、`mamba` 安装；
- HuNavSim 升级、第二套 ROS、Behavior Tree/状态机第三方库；
- 自定义 ROS 消息、PPO observation 修改；
- Isaac/Conda 元数据、D6、碰撞、odom、Nav2 或 Character 代码修改。

V1 实现只可依赖 Python 标准库、underlay 已有的 `rclpy`、ROS 消息包和 PyYAML。

## 8. V1 边界

V1 只交付 `1 Robot + 1 Human`：事件提取、五状态确定性自动机、HuNav profile 切换、
JSON topic/JSONL trace、独立 demo 和基线回归。

状态为 `NORMAL`、`ATTENTION`、`CURIOUS`、`SURPRISED`、`SCARED`。现有
`THREATENING` 只保留在原六行为 demo；`SOCIAL`、行人共享事件、多人、动态
spawn/despawn、ID 复用、RL、概率自动机和 UPPAAL/形式化导出均不属于 V1。

完成实现和测试后，交付记录必须明确区分：自动化通过、需要 GPU 的完整仿真通过、未能
运行及其原因。不得把计划中的期望 marker 或性能门槛写成已经取得的结果。

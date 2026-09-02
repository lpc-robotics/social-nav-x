# social-nav-x 形式化社交自动机 V1：已实现交付与后续开发规范

> 适用分支：`feature/formal-social-automata-v1`
> 唯一源码基线：`51ab117dedf6a8173c1704f0edd8d01c7938fb8e`
> 基线标签：`arena5-isaac5.1-archive-20260829`
> 开发目录：`/home/lpc/workspace/social-nav-x-formal-v1`
> 运行工作区：`/home/lpc/workspace/arena5_ws`（2026-09-02 已定向合并源码；共享
> `install/`、Conda 和依赖仍为只读 underlay，构建产物隔离在 `.colcon-formal-v1`）

本文是 V1 的实现记录、验收合同和后续 Codex Agent 的权威开发文档。原始需求快照位于
`docs/formal_social_automata/source_spec_20260830.md`，经项目实况校正后的接口、范围、
阈值、测试和恢复规则以本文为准。不可变事实和风险边界另见
`docs/formal_social_automata/BASELINE.md`。

## 0. 当前交付状态（2026-09-02）

V1 代码闭环和计划内自动化、真实 HuNav、GPU 仿真及基线回归均已完成。实现阶段提交为：

```text
ec95e8c  docs: record formal automata baseline
eaa84c7  feat: add deterministic social automaton core
e33dd6d  feat: add formal social proxy and demo overlay
a048bfd  test: deliver formal social automata v1
后续修复  Regular 目标处残留运动、可视化模式和 Surprised 渲染朝向验收（以 feature HEAD 为准）
本次修复  ROS +X 与 Isaac People -Y 前向轴转换、Scared 危险优先（以 feature HEAD 为准）
0359579  fix: load character frame overlay in six behavior demo
d8b026d  fix: prefer validated shared arena isaac install
```

已取得的权威结果：

- 独立 overlay 构建成功；纯坐标转换测试 `17/17`、`arena_humble_compat` `20/20`、
  `formal_social_behavior` `76/76` 均通过；两包分别由 `colcon test-result --verbose`
  报告 `20` 和 `261` 个 test/subtest，全部 `0 errors, 0 failures, 0 skipped`。其中包含
  真实 `hunav_agent_manager`、代理 failure/rollback、disabled passthrough、四请求并发和
  bridge 兼容测试，以及目标处 Regular 残留速度和渲染姿态检查。
- 完整 GPU 矩阵位于
  `/home/lpc/workspace/social-nav-x-formal-v1/logs/formal_acceptance/20260901_173536_764353999_pid127944`；
  `matrix.log` SHA-256 为
  `169b55b0c739a36227b7b761314655984262092eaeff0b0e53cd97424db0aecb`，取得
  `FORMAL_SOCIAL_ACCEPTANCE_MATRIX_OK cases=6 rounds=2 scenarios=safe,sudden,fast`。
- safe 两轮均为 `ATTENTION -> CURIOUS -> NORMAL`；sudden 两轮均为
  `ATTENTION -> SURPRISED -> NORMAL`；fast 两轮均为
  `ATTENTION -> SCARED -> NORMAL`。三类 HuNav type `5/3/4` 和各自运动响应均由验收器核对；
  六轮恢复后的单目标 Regular 均为 `regular_motion=stopped`，不再携带特殊行为残留速度。
- 原矩阵两轮 Surprised 的 ROS 逻辑姿态朝向机器人误差为 `2.883°/2.841°`，停止速度均为
  `0.000 m/s`；进一步的 WebRTC 复核确认 Isaac People 资产的视觉前向轴是局部 `-Y`，
  已在 Character 边界增加与 ROS 局部 `+X` 之间的可逆 `+90°/-90°` 转换。
- 12 个互不复用的动作前/后稳态窗口中，HuNav compute 为 `10.882--22.375 Hz`，Isaac
  display 为 `4.678--5.929 Hz`，最大积分步长 `0.025 s`，integration lag 为
  `0.005--0.008 s`，均通过门槛。
- 未 source overlay 的原工作区取得 `SMOKE_NAVIGATION_OK` 和
  `SIX_BEHAVIORS_VERIFY_OK types=1,2,3,4,5,6`；source overlay 后原六行为再次取得同一
  marker。对应日志见 `logs/regression/`。
- 最终 overlay strict 回归位于
  `logs/regression/original_with_character_frame_fix_20260901/`，`arena_isaac` 与 compat 的
  package prefix 都是 feature overlay，并取得
  `SIX_BEHAVIORS_VERIFY_OK types=1,2,3,4,5,6`。
- 经用户授权，formal 源码及 Character 前向轴修复已定向合并到
  `/home/lpc/workspace/arena5_ws`；该目录不是根 Git 仓库，因此运行 manifest 以三个关键
  源文件的组合 SHA-256 `3ac7a64e30f381221cc0835059ac61c43e86961f603e582625bf58d529e7e2f4`
  标识合并内容。原工作区使用独立 `.colcon-formal-v1` 构建成功，`17/17`、`20/20`、
  `76/76` 测试通过；sudden 可视化误差 `2.883°`，原六行为 strict marker 不变。
- 原工作区在合并前同样存在约 `90°` 显示偏差：其实际 `Person.py` SHA-256 为
  `883c1aaec2c242521015313c57f8ce562d5a6122ada27b6c4bcb6a58615df684`，没有
  ROS `+X` 与 Character `-Y` 的边界转换，且 `character_frames.py` 不存在。现已用与
  feature 完全相同的双向转换修复，源码 SHA-256 和验证证据见第 16 节。
- `patches/arena-isaac.patch` SHA-256 为
  `1d404fca247c81a6dfbfaa470cfd35a7ca8005d0b2cd2be056fd9bf141875b23`；在固定上游
  `16b8e3416517d8c3dc1b5038df4fe11b9a6df46c` 上正向和应用后反向
  `git apply --check` 均成功，并逐一比对 31 个应用后文件。

fast 验收采用 `NAVIGATION=false`、唯一 `/cmd_vel` 发布者和 odometry 同步的 `0.8 m/s`
短脉冲。只在 fast 子进程中设置 `physics_dt=0.01 s`、线加速度 `100 m/s²` 和命令看门狗
`0.012 s`；生产 demo 仍为 `1/60 s`、`2.0 m/s²`、`0.5 s`。修正视觉前向轴后，Scared
转身会真实触发 FOV 的 `ROBOT_LOST`；若同拍仍有快速接近/TTC/个人空间危险，安全优先，
自动机保持 `SCARED`，避免 `Regular -> SCARED` reset 风暴。危险解除后的安全
`ROBOT_LOST` 仍可立即恢复。最终 production-style fast 可视化实测只有两次 reset，路径为
`ATTENTION -> SCARED -> NORMAL`，距离上升 `1.146181 m`，退出 closing speed
`-0.839761 m/s`。

## 1. 交付目标与成功定义

V1 最初在不改变活动工作区的隔离 feature 中完成；2026-09-02 经用户授权仅将交付源码
定向合并到活动工作区，继续保持现有六行为、D6、Nav2、共享 install 与 Isaac 依赖不变。
交付的是 `1 Robot + 1 Human` 闭环：

```text
robot/human map-plane kinematics + simulation stamp
                    |
                    v
       deterministic event extractor
                    |
                    v
 NORMAL / ATTENTION / CURIOUS / SURPRISED / SCARED
                    |
                    v
        full HuNav behavior profile
                    |
                    v
   ResetAgents on profile change -> ComputeAgents
                    |
                    v
          HuNav/SFM continuous motion
                    |
                    v
             Isaac Character
```

完成标准：

1. 相同初始上下文、配置及带仿真时间戳的输入序列，始终产生相同事件、状态和
   transition trace。
2. 五状态自动机按本文唯一转移优先级运行，阈值抖动由 Schmitt hysteresis、dwell、
   timeout 和 cooldown 抑制。
3. 自动机只选择 HuNav profile；人物轨迹仍由 HuNav/SFM 计算，代码不直接写人物 pose。
4. HuNav profile 改变时恰好执行必要的 reset/recompute；同 profile 的状态变化不 reset。
5. 独立 formal demo、JSON topic、JSONL transition trace 和测试可用。
6. 原六行为入口和 marker 不变，feature overlay 启用前后都能回归。
7. 活动工作区只有备份清单所列 formal/bridge/Character 文件发生有意源码变更；依赖环境、
   嵌套 Git index 和其余受保护文件不变，可按定向快照恢复合并前版本。

## 2. 范围、非目标与项目事实修正

### 2.1 V1 范围

- 一个机器人和一个固定目标行人，目标 `agent_id=1`。
- 状态：`NORMAL`、`ATTENTION`、`CURIOUS`、`SURPRISED`、`SCARED`。
- 二维事件提取、确定性自动机、HuNav behavior adapter、reset 服务代理。
- 可靠的 `std_msgs/String` JSON 调试 topic、逐行 transition trace。
- 一人 YAML、独立 launch/run script、纯单元测试、服务级测试和仿真验收。
- 对现有六行为 bridge 做默认向后兼容的通用化，原 demo 行为不变。

### 2.2 明确非目标

- `THREATENING` 只保留在既有六行为 demo，不进入 V1 自动机。
- `SOCIAL`、peer/shared events、双人组合自动机、group formation 不实现。
- 不处理动态 spawn/despawn、agent ID 复用或多人 reset 一致性。
- 不实现概率自动机、RL observation/reward、Isaac RL wrapper 或 PPO 修改。
- 不导出 UPPAAL，也不实现任意 YAML guard DSL。
- 不新增自定义 ROS 消息，不升级 HuNavSim，不引入新的 Behavior Tree 或状态机库。
- 不提供基于地图的遮挡、ray tracing、摄像机或 Isaac vision 可见性。

### 2.3 对原始说明的关键修正

- 实际运行工作区不是可直接提交的单仓库，而是包含 11 个独立嵌套仓库及有意未提交
  改动的部署环境。初始实现只在归档仓库的隔离 worktree 开发；2026-09-02 的源码合并
  采用精确 allowlist、合并前快照和独立 build/install/log，不把活动目录误当作单仓库提交。
- HuNav v1 已经内部使用 BehaviorTree.CPP；问题不是“缺少 BT”，而是树只在初始化时
  按 `behavior.type` 创建，后续 `/compute_agents` 中直接改 type 不会动态换树。
- V1 因而采用现有 `/reset_agents` 代理，不修改 HuNav 核心。单人场景接受最多一个
  bridge compute 响应拍的可见切换延迟。
- 当前 DWB `max_vel_x=max_speed_xy=0.26 m/s`；`0.8 m/s` 快速接近场景必须关闭
  Nav2 后用受控 `/cmd_vel`，不能声称由默认 Nav2 复现。
- “可见”仅为距离和行人朝向/FOV；不存在遮挡语义。
- 个人空间进入阈值为项目约定的中心距 `1.0 m`，不是 collider 边缘距离。

## 3. 版本、工作区和依赖保护

开始或续接开发时必须核验：

```bash
cd /home/lpc/workspace/social-nav-x-formal-v1
git rev-parse HEAD
git rev-parse arena5-isaac5.1-archive-20260829^{}
git merge-base HEAD arena5-isaac5.1-archive-20260829
git status --short --branch
sha256sum docs/formal_social_automata/source_spec_20260830.md
```

标签 `arena5-isaac5.1-archive-20260829` 必须解析到
`51ab117dedf6a8173c1704f0edd8d01c7938fb8e`，feature HEAD 必须以它为祖先；形成 feature
提交后，HEAD 本身不再等于基线 commit。来源快照 SHA-256 必须为
`e033334817f3a6096845765969c5af57444644d32d6df618b34de16b9f12a268`。

完整恢复包 SHA-256 为
`69bc5468d9f3e4674fd61eb637242e3744cf8c741a31e1b243892997af2160e0`。灾难恢复只能按
`/home/lpc/workspace/arena5_ws_archives/20260829_164103_full_workspace_pre_log_cleanup/RESTORE.md`
执行，并用同目录的 `KEY_SHA256SUMS` 复核。禁止向现有运行工作区覆盖解压。

保护规则：

- 除 2026-09-02 已授权并记录在第 16 节的定向源码 allowlist 外，不继续编辑
  `/home/lpc/workspace/arena5_ws`；不得在其嵌套仓库运行
  `reset/clean/checkout/pull/rebase`，也不得覆盖用户已有的未提交改动。
- 不运行会写活动工作区或改变 intent-to-add 的原全量 `scripts/build.sh`。
- 不执行 `sudo`、`apt`、`rosdep`、`pip`、`conda`、`mamba` 安装。
- 不改 Isaac 环境、Conda 元数据、lockfile、`.repos`、`upstream/manifest.tsv`。
- `/home/lpc/workspace/arena5_ws/.conda/arena_ros` 和共享 `install/` 仅作为只读
  underlay；feature 使用 `.colcon`，合并后的活动源码使用 `.colcon-formal-v1`，各自持有
  独立 `build/install/log`。
- 只在独立 overlay 构建 `arena_isaac`、`arena_humble_compat` 与
  `formal_social_behavior`；其中 `arena_isaac` 必须来自 feature，确保坐标边界修复生效。
- 不自动 merge `main`，不 push 远端；每阶段通过后再形成独立 commit。

初始源码合并的受保护链路包括活动工作区中的原 `scripts/run_six_behaviors.sh`、六行为
YAML/launch、D6、odom、碰撞、Nav2 和 Isaac/Conda 依赖。随后用户明确要求修复主六行为
入口仍加载旧共享安装的问题；第 17 节记录了仅对 `run_six_behaviors.sh` 的授权例外，其余
链路继续保持不变。活动工作区
`Person.py` 是明确授权的例外：只同步已验证的动画接入修复和可逆姿态坐标边界，并新增
纯函数 `character_frames.py`；合并前内容已单独归档。常规停用只需停止 formal 进程并在
新 shell 中不 source `.colcon-formal-v1`；若要撤销源码合并，必须按第 16 节的定向
`RESTORE.md` 恢复，不得覆盖式解压。

## 4. 代码布局与职责

新增 ROS 2 Python 包位于：

```text
src/formal_social_behavior/
├── formal_social_behavior/
│   ├── model.py
│   ├── config.py
│   ├── event_extractor.py
│   ├── automaton.py
│   ├── behavior_adapter.py
│   ├── proxy_validation.py
│   ├── trace_serialization.py
│   ├── proxy_node.py
│   └── scenario_verifier.py
├── config/
│   ├── formal_social_agent.yaml
│   └── formal_social_automata.yaml
├── launch/formal_social_demo.launch.py
├── test/test_*.py
├── package.xml
└── setup.py

src/arena-isaac/arena_isaac/
├── pedestrian/simulator/logic/people/character_frames.py
└── test/test_character_frames.py
```

包的 console entry point 为：

```text
formal_social_behavior_proxy = formal_social_behavior.proxy_node:main
verify_formal_social_scenario = formal_social_behavior.scenario_verifier:main
```

职责边界：

- `model.py`：Enum/dataclass 值对象，不导入 `rclpy`。
- `config.py`：从 YAML 构造并验证不可变配置，不包含转移逻辑。
- `event_extractor.py`：连续运动快照到指标、latch 和离散事件；不读取 ROS topic。
- `automaton.py`：集中保存可枚举、有顺序的转移拓扑和计时上下文。
- `behavior_adapter.py`：状态到完整 HuNav profile 的纯映射，以及 ROS 消息深拷贝应用。
  纯映射部分不导入 `rclpy`。
- `proxy_validation.py`：在提交前检查 raw response 的数量、顺序、ID/name、type、goal 数量
  和全部有限运动值，不导入 `rclpy`。
- `trace_serialization.py`：稳定 key JSON、无穷 TTC 到 `null`、观察与转移 schema。
- `proxy_node.py`：ROS service、callback 串行化、candidate/commit、topic 和 trace I/O。
- `scenario_verifier.py`：完整仿真的唯一 `/cmd_vel` 验收驱动器；不参与生产闭环。
- `character_frames.py`：标准库实现的 ROS `+X` 前向轴与 Isaac People 资产 `-Y` 前向轴
  四元数双向转换；不导入 Isaac、ROS、NumPy 或 SciPy。
- `formal_social_demo.launch.py` 与包内 YAML：只服务一人 formal 场景，不修改
  `arena-rosnav` 原 launch/YAML。
- 根目录 `scripts/run_formal_social_demo.sh`：独立运行入口和每次运行日志目录。
- 根目录 `scripts/test_formal_simulation_matrix.sh`：每轮使用独立 ROS domain 的两轮三场景验收。
- `arena_humble_compat/hunav_six_behaviors_bridge.py`：只加入默认兼容的 strict/generic
  选择，不承载自动机。

YAML 只配置阈值、计时、目标 agent 和状态到 profile 的映射；转移拓扑固定在代码中，
禁止用字符串表达式、`eval` 或通用 guard DSL。

## 5. 纯核心数据模型

实现使用以下强类型概念：

- `FormalState`：五个状态的 Enum，初态 `NORMAL`。
- `SocialEvent`：`ROBOT_VISIBLE`、`ROBOT_LOST`、`ROBOT_NEAR`、
  `PERSONAL_SPACE_VIOLATION`、`ROBOT_FAST_APPROACH`、`TTC_LOW`、
  `ROBOT_LEAVING`、`SUDDEN_NEAR`、`ROBOT_SAFE_APPROACH`。
- `PlanarKinematics` 与 `MotionSnapshot`：stamp 和 robot/human 二维位置、速度、yaw、半径；
  目标 agent ID 位于 `FormalSocialConfig` 和 ROS 消息，而不重复放进运动值对象。
- `InteractionMetrics`：distance、robot bearing、relative/FOV bearing、closing speed、TTC。
- `EventMemory`：六个 Schmitt latch 与最后 stamp；与自动机计时上下文分开提交。
- `EventSnapshot` / `EventEvaluation`：本拍事件指标以及 candidate memory。
- `AutomatonContext`：状态、进入/安全/cooldown 时间、最后 stamp 与最后转移 stamp。
- `Transition`：stamp、old/new state、cause；agent ID/name 在序列化 ROS 层注入。
- `BehaviorProfile`：type、初始化 seed `state=0`、configuration、duration、once、vel、dist
  和四个 force factor 的完整 tuple。
- `AutomatonStep`：candidate context、事件快照、可选 transition 与 clock-reset 标记；ROS
  代理另行选择目标 profile，只有 raw compute 成功后才同时提交 event memory/context/profile。

核心对象不得依赖 ROS executor、wall clock 或全局可变单例。配置在启动时一次性解析并
验证：未知 wrapper/root/threshold/timing/profile key 直接失败，目标 ID 和 behavior type
必须是未经有损转换的整数，bool、数值字符串和小数均不能冒充。除有符号的 leaving
阈值外，距离/速度/TTC 阈值非负；enter/exit 关系正确，half-FOV 在 `(0,180]`，计时
非负。profile state 名不区分大小写，但归一化后重复也必须失败；mapping 必须恰好覆盖
五状态，且 HuNav seed 固定为 `state=0/configuration=1`、behavior type 合法。运动输入
包含 NaN/Inf、半径为负或目标 ID 缺失时，该次服务失败且上下文不提交，不允许把非法值
写入 HuNav。

形式化状态绝不能存入或读取 `AgentBehavior.state`。profile 中的 `state=0` 只是在 reset
重建 HuNav tree 时使用的初始化 seed；adapter 每拍应用完整 profile，但当前 HuNav
`updateAgents()` 不会把请求 state 回写到内部 BT 运行态。只有 `AutomatonContext.state`
是离散模型 authority。

## 6. 事件提取规范

### 6.1 几何和 TTC

所有量在 `map` 的二维平面计算。定义：

\[
r=p_{robot}-p_{human},\qquad
v_{rel}=v_{robot}-v_{human},\qquad
d=\lVert r\rVert
\]

当 `d>0` 时，闭合速度：

\[
c=-\frac{r\cdot v_{rel}}{d}
\]

`c>0` 表示机器人接近，`c<0` 表示离开；`d=0` 时使用安全的有限约定计算闭合速度，
同时 TTC 必为 `0`。

碰撞时间使用机器人与行人圆盘半径之和
`R=radius_robot+radius_human`，求：

\[
\lVert r+v_{rel}t\rVert=R
\]

展开为 `a*t^2+b*t+c0=0`，其中 `a=v_rel·v_rel`、`b=2*r·v_rel`、
`c0=r·r-R^2`。若已重叠则 TTC 为 `0`；否则只有在正在闭合、判别式非负且存在非负根
时取最小非负根，静止、远离或不相交时为数学无穷。写 JSON 时无穷 TTC 编码为
`null`，不得输出非标准 `Infinity`。

行人朝向向量为 `(cos(human_yaw), sin(human_yaw))`。FOV angle 是该向量与 `r` 的最小
夹角，正确处理 `-pi/pi` 环绕。可见性不检查墙体或场景遮挡。

### 6.2 Schmitt hysteresis 默认配置

| Latch | 进入条件 | 退出条件 |
| --- | --- | --- |
| visible | `d <= 6.0 m` 且 FOV angle `<= 100°` | `d >= 6.5 m` 或 angle `>= 110°` |
| near | `d <= 2.5 m` | `d >= 2.8 m` |
| personal space | `d <= 1.0 m` | `d >= 1.2 m` |
| fast approach | `c >= 0.50 m/s` | `c <= 0.35 m/s` |
| low TTC | `TTC <= 1.5 s` | `TTC >= 2.0 s` 或 TTC 无穷 |
| leaving | `c <= -0.10 m/s` | `c >= 0 m/s` |

进入/退出边界使用表中的包含关系；位于 enter/exit 之间时保持上一已提交 latch。初始
latch 全为 false，第一帧直接按进入条件计算。

离散事件语义：

- `ROBOT_VISIBLE`、`ROBOT_NEAR`、`PERSONAL_SPACE_VIOLATION`、
  `ROBOT_FAST_APPROACH`、`TTC_LOW`、`ROBOT_LEAVING` 是对应 latch 为 true 时的
  level event。
- `ROBOT_LOST` 只在 visible 从 true 变为 false 的一帧出现。
- `SUDDEN_NEAR` 只在 near 从 false 变为 true 的一帧出现，并且
  `0.25 <= c < 0.50 m/s`，同时 personal-space、low-TTC、fast-approach 全为 false。
- `ROBOT_SAFE_APPROACH` 是派生 level event：near 为 true、
  `0 <= c < 0.25 m/s` 且三类危险均为 false。

“危险”定义为 personal-space、low-TTC、fast-approach 三者任一为 true。“安全闭合”
用于 `ATTENTION -> CURIOUS`，等价于 `ROBOT_SAFE_APPROACH`。
“连续安全”只在 `SURPRISED`/`SCARED` 的当前驻留期内累计：进入这两个状态时先清空旧
recovery timer；若进入 `SURPRISED` 的样本已经安全，则从状态进入 stamp 开始；进入
`SCARED` 后从危险全部退出的首个已提交 stamp 开始。任何后续危险样本都会再次清除
该计时。不得沿用进入状态前已经累积的安全时长。

## 7. 确定性自动机

默认计时：

```text
attention_dwell = 0.5 s
recovery_timeout = 3.0 s
reentry_cooldown = 0.5 s
```

每个状态严格按下表从上到下匹配，第一个 true guard 胜出，同一请求最多产生一次状态
变化：

| 当前状态 | 顺序 | Guard | 下一状态 / Cause |
| --- | ---: | --- | --- |
| NORMAL | 1 | 任一危险事件 | SCARED / 最高优先级的危险 cause |
| NORMAL | 2 | `SUDDEN_NEAR` | SURPRISED / SUDDEN_NEAR |
| NORMAL | 3 | cooldown 已结束且 `ROBOT_VISIBLE` | ATTENTION / ROBOT_VISIBLE |
| ATTENTION | 1 | 任一危险事件 | SCARED / 危险 cause |
| ATTENTION | 2 | `ROBOT_LOST` | NORMAL / ROBOT_LOST |
| ATTENTION | 3 | `SUDDEN_NEAR` | SURPRISED / SUDDEN_NEAR |
| ATTENTION | 4 | 已驻留 `0.5 s` 且 `ROBOT_SAFE_APPROACH` | CURIOUS / ATTENTION_DWELL |
| CURIOUS | 1 | 任一危险事件 | SCARED / 危险 cause |
| CURIOUS | 2 | `ROBOT_LOST` | NORMAL / ROBOT_LOST |
| CURIOUS | 3 | `ROBOT_LEAVING` 且 near 已退出 | NORMAL / ROBOT_LEAVING |
| SURPRISED | 1 | 任一危险事件 | SCARED / 危险 cause |
| SURPRISED | 2 | `ROBOT_LOST` | NORMAL / ROBOT_LOST |
| SURPRISED | 3 | 连续安全满 `3.0 s` | NORMAL / RECOVERY_TIMEOUT |
| SCARED | 1 | `ROBOT_LOST` 且当前无危险事件 | NORMAL / ROBOT_LOST |
| SCARED | 2 | `ROBOT_LEAVING`、near 已退出且连续安全满 `3.0 s` | NORMAL / RECOVERY_TIMEOUT |

同一时刻多个危险事件的 cause 优先级固定为：

```text
PERSONAL_SPACE_VIOLATION > TTC_LOW > ROBOT_FAST_APPROACH
```

状态进入和计时规则：

- 首次有效输入从 `NORMAL` 开始评估；`NORMAL -> ATTENTION` 不需要先等待 dwell。
- attention dwell 从进入 `ATTENTION` 的已提交 stamp 起算。
- recovery 只按连续安全时长计算，不能把状态总驻留时长当作安全时长。
- Scared 的逃离转身可能让 FOV 在危险仍活跃时产生 `ROBOT_LOST` 边沿；该组合样本保持
  `SCARED` 且清空安全计时，不提交 reset。后续安全的 lost 样本仍按表恢复。
- 每次进入 `NORMAL` 都从该 stamp 启动 reentry cooldown；危险升级不受 dwell/cooldown
  限制。
- 重复 stamp 不推进 timer；一个 stamp 已提交过 transition 后，同 stamp 后续请求不能
  再提交第二个 transition。
- 仿真 stamp 小于上次已提交 stamp 时视为 time rollback：清除所有 latch/timer（包括
  cooldown），恢复 `NORMAL/Regular`，必要时通过同一 reset transaction 重建 HuNav；
  若旧状态不是 `NORMAL`，transition cause 为 `TIME_RESET`。rollback 后第一份
  非回拨样本可立即重新进入 `ATTENTION`；检测到回拨的那一拍本身只负责恢复
  `NORMAL/Regular`，不会同拍再做第二次转移。

转移拓扑必须集中为可枚举表，并提供检查：所有 state/cause 有定义、同一有序事件输入
只取一个 guard、无意外 dead end。V1 不要求生成形式化工具模型。

## 8. HuNav behavior profile 映射

全部 profile 使用 custom configuration `1` 和 force factors
`goal=2.0`、`obstacle=10.0`、`social=5.0`、`other=20.0`：

| Social state | HuNav type | duration | once | vel | dist |
| --- | --- | ---: | --- | ---: | ---: |
| NORMAL | Regular (`1`) | `40.0` | `true` | `0.6` | `0.0` |
| ATTENTION | Regular (`1`) | `40.0` | `true` | `0.6` | `0.0` |
| CURIOUS | Curious (`5`) | `30.0` | `false` | `0.8` | `1.5` |
| SURPRISED | Surprised (`3`) | `30.0` | `false` | `0.6` | `4.0` |
| SCARED | Scared (`4`) | `40.0` | `false` | `0.6` | `3.0` |

`NORMAL` 和 `ATTENTION` 的 profile tuple 完全相同，所以二者转移不得 reset。profile
比较必须覆盖表中全部字段和四个 force factors，不能只比 type。

adapter 深拷贝 `ComputeAgents.Request.current_agents`，按 ID 精确找到唯一目标，保留
pose、yaw、goals、半径及其他 agent 字段，只覆盖完整 behavior profile。唯一运动字段
例外见 9.2：目标 profile 为 Regular、首目标已进入 HuNav 的 `goal_radius + 0.1 m` 判定
范围且消息仍带非零速度时，只在深拷贝中把全部 linear/angular motion 清零。输入消息不被
原地修改。配置的 `behavior.state` 固定为 `0` 并属于完整 profile
signature；HuNav 在 reset 后以它初始化内部 agent，随后由内部 BT 维护自己的 state。
形式化层从不以该字段推导 `FormalState`。

配置模型采用：

- `EventThresholds`：所有 `*_enter/*_exit` 距离/FOV/speed/TTC 及 sudden-near 上下界；
- `AutomatonTiming`：`attention_dwell_seconds`、`recovery_timeout_seconds`、
  `reentry_cooldown_seconds`；
- `FormalSocialConfig`：`target_agent_id`、thresholds、timing、behavior profiles。

配置文件支持 `formal_social_behavior.ros__parameters` 下的 `thresholds`、`timing`、
`behavior_profiles`，profile key 必须是五个大写状态名；纯测试也可直接传同结构 mapping。

## 9. ROS 服务代理与事务语义

### 9.1 服务拓扑

formal launch 中，bridge 仍调用原类型和名称：

```text
bridge --hunav_msgs/srv/ComputeAgents--> /compute_agents (proxy)
```

HuNav manager 的服务 remap 为：

```text
/compute_agents -> /formal_social_behavior/compute_agents_raw
/reset_agents   -> /formal_social_behavior/reset_agents_raw
```

proxy 创建对应 raw client。`enabled` 默认 `false`；formal launch 显式设为 `true`。关闭
时只把 compute 请求转发给 raw compute，不提取/提交事件、不发布自动机 JSON、不调用
reset。

### 9.2 单请求 candidate/commit

启用时 `/compute_agents` server 使用 `MutuallyExclusiveCallbackGroup` 串行执行；raw
clients 使用独立 `ReentrantCallbackGroup`：

1. 验证 stamp、目标 ID 和有限运动数据，从上次已提交 event memory/context 生成
   `EventEvaluation` 与 `AutomatonStep` candidate。
2. 深拷贝请求，并把 candidate state 的完整 profile 应用于目标 agent；若 candidate 为
   Regular 且首目标已在 `goal_radius + 0.1 m` 内，把副本的全部线/角运动字段清零。
3. 若 candidate profile tuple 与上次已提交 profile 不同：
   - 构造 `ResetAgents.Request`，携带修改后的当前 agents 和原 robot；
   - 调用 raw reset 并要求非空响应且 `ok=true`；
   - reset 成功后再进入第 4 步。
4. 调用 raw compute，参数为同一份修改后当前 agents、原 robot 和原 simulation stamp。
5. 只有 raw compute 返回数量、顺序、ID/name、behavior type、goal 数和有限值全部合法的
   `updated_agents` 后，才生成返回副本；若这一拍 Regular 首次进入同一目标判定范围且
   raw response 仍有非零运动，也在返回副本中清零，再提交 candidate context/profile。
6. 把同一 ROS service 类型的 response 返回给 bridge，不改变 pose、yaw、goal 或 wire
   type；上述严格目标边界内的运动归零是唯一修正。
7. commit 后 best-effort 发布 state/events/transition 并 append/flush JSONL；telemetry
   异常只记录错误，不能把已经成功的 raw compute 变成 bridge 重试而造成重复计算。

reset 或 compute 超时、future exception、reset `ok=false`、返回 agent 数量/ID 非法时，不
提交 candidate，不发布成功事件/状态/transition，不写 transition trace，让 bridge 沿
既有失败路径重试。reset 成功但 compute 失败时，下一请求从旧 formal context 重新求值
并允许再次 reset；不能假装切换已经成功。

node 使用四线程 `MultiThreadedExecutor`；从 service callback 同步等待 raw future 时，
reentrant client callback 仍可被其他线程调度。mutually-exclusive service group 保证两个
compute callback 不会并行进入；显式 `threading.Lock` 再对完整 candidate/reset/compute/
commit transaction 做串行保护。四请求并发测试证明该锁不会阻塞独立 reentrant raw
future callback，也不会出现 reset/compute 配对交错。

首次请求用请求中实际 profile 与 candidate profile 比较：已经是 Regular 时不 reset，
若第一拍直接进入非 Regular 则必须 reset。signature 使用 ROS `float32` wire 精度规范化，
避免 `0.6` 与 `0.6000000238` 产生伪变化；`NORMAL -> ATTENTION` 因完整 tuple 相同不
reset。仿真时间回拨会清除 latch/timer 并恢复 `NORMAL`，但只有当前 profile 不是
Regular 时才需要 reset。

该归零用于修复 HuNav v1 单一 cyclic goal 的确定性边界缺陷：`BTRegularNav` 在
`IsGoalReached` 成功后只执行 `UpdateGoal`；单个 goal 会被旋转回自身，`RegularNav` 不再
tick，也不会清除 Curious/Surprised/Scared 留下的 velocity。旧实现因此可同时报告固定
pose 与非零 velocity，Isaac 每个 display tick 被重新锚定到同一 pose，却继续播放 walk
animation。代理不改 HuNav 依赖、不直接移动人物，只把“已到目标”的运动报告恢复为一致
的零速度；非目标范围的 Regular 运动完全透传。

## 10. 可观测性与 trace 契约

使用 reliable、volatile、depth `10` 的 `std_msgs/msg/String`：

```text
/formal_social_behavior/states
/formal_social_behavior/events
/formal_social_behavior/transitions
```

JSON 使用 UTF-8、稳定 key、`allow_nan=false`，共同字段至少为：

```json
{
  "schema_version": 1,
  "sim_time_ns": 12300000000,
  "agent_id": 1,
  "agent_name": "formal_human",
  "state": "ATTENTION",
  "events": ["ROBOT_VISIBLE"],
  "distance": 2.7,
  "closing_speed": 0.15,
  "ttc_seconds": null,
  "behavior_type": 1,
  "reset_count": 0,
  "config_sha256": "..."
}
```

`ttc_seconds=null` 表示数学无穷；有限 TTC 为 JSON number。transition JSON 另含
`message_type`、`old_state`、`new_state`、`cause`。events 数组按事件字符串字典序稳定
排序，不能依赖 set 的随机迭代次序；完整 payload 还包含两个 bearing。

state/events 在每次成功提交的 compute 后发布；transition 仅在真实状态变化后发布。
JSONL 文件每行与 transition topic 使用同一 schema 和值，写入后 flush。run script 为
每次运行创建不重用的目录，例如：

```text
<worktree>/logs/formal_social/<timestamp_ns>_formal_social_gpu3_pid<PID>/
```

目录中记录 `console.log`、`transitions.jsonl`、`run_manifest.txt`，后者包含生效配置
路径/SHA-256、commit、GPU、navigation、ideal chassis、physics dt、acceleration 和
ideal-command timeout。大型运行日志不提交 Git；交付文档只记录其绝对位置、配置哈希
和精简摘要。

## 11. bridge 向后兼容与 formal demo

`hunav_six_behaviors_bridge` 新增以下参数：

| 参数 | 默认值 | formal demo |
| --- | --- | --- |
| `strict_six_behavior_demo` | `true` | `false` |
| `status_topic` | `/arena5/six_behaviors/status` | `/formal_social_behavior/bridge_status` |
| `agent_debug_topic` | `/arena5/six_behaviors/agents` | `/formal_social_behavior/hunav_agents` |
| `ready_marker` | `SIX_BEHAVIORS_READY` | `FORMAL_SOCIAL_BRIDGE_READY` |

strict 模式必须精确保留：六个 agent、唯一 types `1..6`、custom configuration、六个
Character model、原 topic、原 marker 和原 verifier 行为。generic 模式使用 YAML 中
实际 agent 数量，验证 ID/name 唯一、behavior type 在 `1..6`、每个 agent 恰有一个
Character model；所有 spawn/update/result 数量检查使用实际计数。

原 `SIX_BEHAVIORS_RUNNING` marker 的既有字段保持不变，并追加基于相邻报告区间的
`steady_compute`/`steady_display`。验收只接受动作前和动作后两个不同、计数严格递增的
报告，不能重复使用累计平均值掩盖服务停滞；strict 默认行为与 20 Hz wheel command timer
均未改变。

formal demo 的一人 YAML 和 launch 全部安装在 `formal_social_behavior` 包中；不修改
`arena-rosnav` 的原 YAML/launch。launch 启动原 HuNav loader/manager、proxy 和通用模式
bridge，并只在该 launch 内做 raw service remap。Regular 的唯一 goal 等于出生点，使
人物在受控场景开始前保持静止；Curious/Surprised/Scared 仍由 HuNav 特殊 BT 驱动。

根脚本 `scripts/run_formal_social_demo.sh` 使用独立日志目录，默认：

```text
GPU_ID=3
NAVIGATION=false
ARENA_IDEAL_CHASSIS=true
```

只有显式请求时才启用 Nav2。任何受控 `/cmd_vel` 场景必须确认没有第二个 publisher；
脚本不得改写原 `scripts/run_six_behaviors.sh` 或复用其 marker 伪装 formal 成功。生产
入口默认 physics dt/ideal linear acceleration/command timeout 仍是
`1/60 s`、`2.0 m/s²`、`0.5 s`；只有仿真矩阵 fast 短脉冲在子进程设置
`0.01 s`、`100.0 m/s²`、`0.012 s`，且写入每次 run manifest。

新增 `scripts/run_formal_social_visual_scenario.sh` 作为第二终端的可视化驱动入口。它不
另启 Isaac、不改变自动机 guard，只连接已经运行的 formal demo，确保自己是唯一
`/cmd_vel` publisher，并开启 verifier 的 `visual_mode`。每次状态/转移和每 `0.5 s` 的
观察样本会打印 state、behavior type、distance、speed、ROS 逻辑姿态四元数换算的
`human_yaw_deg`、机器人方向 `target_yaw_deg` 与 `facing_error_deg`；Character Graph 使用
同一姿态加 `+90°` 资产轴偏移，因此局部 `-Y` 视觉前向与该逻辑 yaw 一致。输出和 manifest 写入
独立 `logs/formal_visual/<timestamp>_<scenario>_pid<PID>/`。可选 hold 只在目标状态期间
持续发布零机器人速度，最长 `60 s`；自动机自身发生恢复时会提前结束，不冻结仿真时间。

## 12. 测试与验收结果

### 12.1 纯 Python 单元测试

不 source ROS 即可测试的核心覆盖已经实现：

- 距离、yaw 环绕/FOV、闭合速度的 approaching/stationary/leaving。
- 圆盘 TTC：正碰、掠过不相交、静止、远离、已重叠、不同半径。
- visible/near/personal-space/fast/TTC/leaving 的全部 enter、hold、exit 边界。
- `SUDDEN_NEAR` 仅为 near 进入边沿，速度上下界和危险抑制正确。
- 安全接近：`NORMAL -> ATTENTION -> CURIOUS -> NORMAL`。
- 突发近距：`NORMAL/ATTENTION -> SURPRISED -> NORMAL`。
- 危险：各非 SCARED 状态优先升级，多个危险 cause 顺序确定。
- Scared 同拍 `ROBOT_LOST + danger` 保持 Scared；危险解除后的安全 lost 正常恢复。
- dwell、连续安全 recovery、cooldown、重复 stamp、时间回拨。
- 每 stamp 最多一次 transition，转移表无未知 state/cause 和非确定性。
- behavior adapter 深拷贝，不修改原请求；完整 profile 比较和 state 字段规则正确。
- HuNav 单目标 Regular 的 `goal_radius + 0.1 m` 边界、完整运动归零深拷贝，以及非目标
  Regular 不受影响。
- 同一录制输入离线重放两次产生字节级稳定的 transition JSONL。
- 配置缺键、关系反转、未知 state、NaN/Inf 和重复 agent ID 失败明确。
- 配置拼写错误、分数/字符串/bool agent ID、缺状态 profile 及非零 HuNav seed 均被拒绝，
  不允许静默回落为默认值。
- ROS/Character 四元数双向转换覆盖 `-π..π`、单位化、零/NaN 拒绝，并直接验证 Isaac
  People 局部 `-Y` 前向经转换后对齐 ROS yaw。

### 12.2 服务级测试

使用真实 HuNav manager、但不启动 Isaac：

- 初始 `NORMAL/Regular`，第一次安全可见只到 `ATTENTION` 且不 reset。
- `ATTENTION -> CURIOUS`、`-> SURPRISED`、`-> SCARED` 以及恢复 Regular 时，每次
  profile 变化恰好一个成功 reset。
- `NORMAL <-> ATTENTION` 及同状态重复 compute 不 reset。
- reset 返回 `ok=false` 或 compute 返回空/非法 response 时，formal
  state/profile/trace 均不提交；代理本身另有有界 service wait，超时走同一不提交路径。
- raw response service type 不变，agent 数量、ID、pose、velocity、goals 全部有限且保持。
- 特殊行为恢复 Regular 时，进入 reset/compute 的副本及最终 response 在已到目标时全部
  motion 字段为零；raw compute 首次进入目标半径时也清除最后一拍运动，调用者请求不变。
- `enabled=false` 完全透传且不产生 formal side effect。
- 两个同时到达的 compute 请求保持 `reset(A)-compute(A)-reset(B)-compute(B)` 配对，
  不会交错；延迟 raw compute 超时后 candidate 仍不提交。

串行性由 `/compute_agents` 的 `MutuallyExclusiveCallbackGroup` 直接保证，raw future 则由
独立 `ReentrantCallbackGroup` 和多线程 executor 完成；服务测试显式覆盖并发配对和
超时 rollback，完整 GPU 运行再核对没有持续 future 积压。

最终结果：纯核心、adapter/trace/config/validation、mock proxy 与真实 HuNav 测试共同由
colcon 收集；详细总数见 12.3。

### 12.3 独立 overlay 构建

在 activity underlay 之上、worktree 内执行选择性 build/test；命令必须显式指定独立
base paths，不能写共享 install。最低结果：

```text
colcon build: arena_isaac + arena_humble_compat + formal_social_behavior 成功
direct pytest: character_frames 17 passed
colcon test: compat/formal 两包相关测试无失败
colcon test-result --verbose: 0 failures
```

实际使用 `scripts/build_formal_overlay.sh` 和 `scripts/test_formal_overlay.sh` 完成；构建上述
三包。Isaac Kit 的完整 `arena_isaac` 测试依赖 `omni` 运行时，普通 ROS Python 不能导入；
因此测试脚本只对新增的纯转换模块运行定向 pytest，再用 colcon 测 compat/formal：

```text
character_frames: 17 passed
arena_humble_compat: 20 passed
formal_social_behavior: 76 passed
colcon test-result (compat): 20 tests, 0 errors, 0 failures, 0 skipped
colcon test-result (formal): 261 tests, 0 errors, 0 failures, 0 skipped
```

三组 pytest 共收集 113 个顶层 case；formal 的 unittest subtests 展开后为 261。最终
colcon 日志位于 `.colcon/test-log/test_2026-09-01_21-12-30/`，仅有两条依赖侧 Lark
deprecation warning。`ros2 pkg prefix arena_isaac` 与
`ros2 pkg prefix formal_social_behavior` 均解析到 feature `.colcon/install/`。构建生成物
未提交，也未安装或升级依赖。

### 12.4 完整一人仿真

以下场景均由 `scripts/test_formal_simulation_matrix.sh` 从干净 formal run 开始并重复两轮：

矩阵不是只打印观测值：每个 case 在动作前等待真实 `SIX_BEHAVIORS_RUNNING`，硬性解析并
检查 compute/display/max-dt/integration-lag；动作驱动器检查唯一 `/cmd_vel` publisher、
有限 ROS 数据、目标/恢复 reset count、重复 transition 和恢复后 pose/velocity 一致性；
单目标 demo 必须得到 `regular_motion=stopped`，不能再接受固定 pose 加非零速度。每次
cleanup 前后扫描
Traceback、异常进程退出和明确的 proxy/bridge service error。矩阵开始和结束还分别校验
归档 `KEY_SHA256SUMS` 与 10 个 Person/D6/碰撞/六行为/Nav2 守护文件哈希。

| 场景 | 控制 | 期望 |
| --- | --- | --- |
| 安全接近 | `0.15 m/s` | `NORMAL -> ATTENTION -> CURIOUS` |
| 突发近距 | 以 `0.30 m/s` 首次进入 near，且 TTC/个人空间安全 | 进入 `SURPRISED` |
| 快速接近 | `NAVIGATION=false`、单一控制器 `0.8 m/s` | 进入 `SCARED` |
| 离开/恢复 | 机器人离开并满足各状态 guard | 回到 `NORMAL/Regular` |

最终矩阵的六个 marker、路径和关键数值：

| 场景 | 两轮结果 |
| --- | --- |
| safe | target `2.497/2.499 m`，recovery `2.818/2.805 m`，`regular_motion=stopped`；两轮均 `ATTENTION->CURIOUS->NORMAL` |
| sudden | target `2.490/2.491 m`，recovery `4.175/4.156 m`，朝向误差 `2.883°/2.841°`；两轮均 `ATTENTION->SURPRISED->NORMAL` |
| fast | target `3.016/3.008 m`，recovery `4.080/3.009 m`，`regular_motion=stopped`；两轮均 `ATTENTION->SCARED->NORMAL` |

原矩阵的 fast 数值继续作为闭环性能基线。坐标修复后的补充 fast 实测要求 type 4、
SCARED 区间 distance 上升、closing speed `< -0.01 m/s`，并从相同 simulation stamp 的
odom/state 重建 `human_outward = robot_toward_human - closing_speed`。Scared 转身产生的
`ROBOT_LOST` 若与危险同拍则不恢复；最终只有 `ATTENTION->SCARED` 和
`SCARED->NORMAL:RECOVERY_TIMEOUT` 两次 profile reset，不再出现 Regular/Scared reset
风暴，也没有直接写机器人或人物 pose。

Surprised 验收使用 `/human_states.position.orientation` 的 ROS `x,y,z,w` 四元数换算逻辑
yaw，不再把可能滞后的 `Agent.yaw` 当作唯一 authority。`Person.py` 在 Character Graph
入口执行 `q_character = q_ros * qz(+π/2)`，因为 Isaac People 资产局部 `-Y` 才是视觉
前向；读取 graph transform 时执行 `q_ros = q_character * qz(-π/2)`，所以 ROS feedback
不携带资产偏移。通过条件仍是速度 `<=0.02 m/s`、逻辑朝向机器人误差 `<=3°`、
`Agent.yaw` 与 ROS 四元数 yaw 误差 `<=0.25°`，并且相对初始 yaw 至少变化 `0.20 rad`。

权威矩阵目录与哈希：

```text
/home/lpc/workspace/social-nav-x-formal-v1/logs/formal_acceptance/20260901_173536_764353999_pid127944
matrix.log SHA-256: 169b55b0c739a36227b7b761314655984262092eaeff0b0e53cd97424db0aecb
```

六个 case 各有独立 domain、launch/verifier log、run manifest 和 transition JSONL；对应
formal run 目录按 safe1/safe2/sudden1/sudden2/fast1/fast2 为：

```text
/home/lpc/workspace/social-nav-x-formal-v1/logs/formal_social/20260901_173537_801800213_formal_social_gpu1_pid128013
/home/lpc/workspace/social-nav-x-formal-v1/logs/formal_social/20260901_173643_314239505_formal_social_gpu1_pid132618
/home/lpc/workspace/social-nav-x-formal-v1/logs/formal_social/20260901_173744_897240554_formal_social_gpu1_pid137111
/home/lpc/workspace/social-nav-x-formal-v1/logs/formal_social/20260901_173847_721796332_formal_social_gpu1_pid141604
/home/lpc/workspace/social-nav-x-formal-v1/logs/formal_social/20260901_173954_591624466_formal_social_gpu1_pid146242
/home/lpc/workspace/social-nav-x-formal-v1/logs/formal_social/20260901_174114_351433866_formal_social_gpu1_pid151697
```

运行验收：

- steady-state wall-clock HuNav compute `>=10 Hz`；
- Isaac display `>=4.5 Hz`；最大 integration step `<=0.026 s`；
- 无持续 future 积压、service error、NaN/Inf 或同 profile 重复 reset；
- Curious 时人机距离下降；Scared 产生向外径向速度/负闭合速度；Surprised 速度归零且
  Character 的局部 `-Y` 视觉前向在 `3°` 内转向机器人；恢复 Regular 后速度/位移一致；
- 人物 pose 仍由 HuNav 结果驱动；初始 feature 验收阶段只有 overlay 的 `Person.py` 增加
  姿态边界转换。2026-09-02 定向合并后，活动工作区 `Person.py` 也包含同一转换；D6、
  碰撞、Nav2 和共享依赖仍未改变。

最终 12 个互不复用的动作前/后窗口实测 compute `10.882--22.375 Hz`、display
`4.678--5.929 Hz`、max step `0.025 s`、lag `0.005--0.008 s`；每个 case 均有且仅有
3 条转移、2 条真实 manager reset log 和 1 条成功 marker。

### 12.5 可视化模式验收

启动 WebRTC formal demo 后，在第二终端运行：

```bash
cd /home/lpc/workspace/social-nav-x-formal-v1
scripts/run_formal_social_visual_scenario.sh safe
scripts/run_formal_social_visual_scenario.sh sudden 2.0
scripts/run_formal_social_visual_scenario.sh fast 2.0
```

每次只运行一个驱动器；`safe` 默认最多观察 `8 s`，`sudden/fast` 默认 `2 s`，第二参数可
覆盖为 `[0,60]` 秒。可视化模式仍执行完整自动验收并最终打印
`FORMAL_SOCIAL_SCENARIO_OK ... visual_mode=true`，失败时非零退出，不能用画面主观判断
代替数值 gate。

2026-09-01 在 GPU 3 的真实 Isaac/HuNav production-style 运行中完成坐标修复后的三模式
补充验收：

- sudden：逻辑 yaw 从 `177.14°` 稳定到目标约 `179.97°`，速度 `0`，最终误差
  `2.862°`；Character 局部 `-Y` 前向经 `+90°` 边界转换后在 WebRTC 中面对机器人。
- safe：进入 Curious 后 distance 在观察窗内从 `2.480 m` 连续下降到 `2.333 m`，证明
  转换没有破坏跟随位移；恢复 Regular 后停止。
- fast：Scared 时逻辑 yaw 约 `0°`，与机器人 bearing 约 `180°` 相反；distance 上升
  `1.146181 m`，退出 closing speed `-0.839761 m/s`，两次 reset，无重复切树。

三次均取得 `FORMAL_SOCIAL_SCENARIO_OK ... visual_mode=true`。原始日志及 SHA-256：

```text
logs/formal_visual/20260901_194912_743233044_sudden_pid548382/visual.log  a10f4a3a60f58a438b75416cc455e40ed6e91cc63bf57ee738d9842585b6a694
logs/formal_visual/20260901_195144_636426696_safe_pid557741/visual.log    45f211e5ffa60707d7994c0adc8e6cad568542025e6be6017becf4215738e13f
logs/formal_visual/20260901_210908_750275095_fast_pid795427/visual.log    2bd9cb5bc96c1df699e22370eb719ee5f392fe07eaa3a93ff86dd38d19ed13a1
```

### 12.6 基线回归

最终已完成两组：

1. 新 shell 不 source feature overlay，运行原活动工作区入口，取得
   `SIX_BEHAVIORS_VERIFY_OK types=1,2,3,4,5,6` 和 `SMOKE_NAVIGATION_OK`。
2. source feature overlay 后运行原六行为入口，再次取得相同六行为 marker，证明 bridge
   参数默认值完全兼容。

实际原工作区 marker：

```text
SMOKE_NAVIGATION_OK start=(3.000,3.000) end=(4.881,2.884) moved=1.885m lidar_messages=96
SIX_BEHAVIORS_VERIFY_OK types=1,2,3,4,5,6 active=3,5 responses=3,4,5,6
```

overlay 后最终回归目录为
`/home/lpc/workspace/social-nav-x-formal-v1/logs/regression/original_with_character_frame_fix_20260901/`；
`arena_isaac_prefix.txt` 和 `overlay_prefix.txt` 证明两个修改包都解析到 feature overlay，
marker 为：

```text
SIX_BEHAVIORS_VERIFY_OK types=1,2,3,4,5,6 active=3,5 responses=3,4,5,6 robot_distance=1.035 robot_states=302
```

该坐标修复后 strict run 的独立窗口 compute 为 `14.336--15.552 Hz`、display 为
`4.738--4.874 Hz`，`max_dt=0.025 s`、lag `0.008 s`；验收后通过 Ctrl-C 清洁退出。
原工作区既有 no-overlay 稳定段约 `16.0--16.5 Hz` compute / `4.8--4.9 Hz` display。

2026-09-02 源码合并后的底盘/碰撞状态必须按实际执行记录理解：因为活动工作区
`Person.py` 被有意修改，最初启动了底盘矩阵；首次 case 1 因 `/SpawnUrdf` 启动时序失败，
没有生成测量结果。重试目录中最终落盘 `18/22` 条记录，18 条均 `valid=true`；之后用户
明确要求跳过底盘和碰撞测试，因此剩余 `4/22` **未执行**，碰撞套件为 `0/3`
**未执行**。这些项目没有被宣称为完整通过，也不再自动续跑。部分证据和哈希见第 16 节；
D6/碰撞源码未被本次合并修改，历史完整 `22/22`、`3/3` 基线仍可参考，但不能替代本次
未执行项。formal 测试的两条 warning 仍只是依赖侧 Lark deprecation warning。

## 13. 实施顺序、提交与补丁维护

初始 V1 按四个可审查提交交付；后续两个视觉问题修复各自作为独立可回退提交，最终仍以
feature HEAD 为交付点：

1. `ec95e8c` **文档基线**：原稿快照与 `BASELINE.md`。
2. `eaa84c7` **纯自动机**：model/config/event extractor/automaton/adapter/trace 及纯测试。
3. `e33dd6d` **ROS 代理与 demo**：proxy、validation、YAML、launch/run script、bridge
   通用模式、mock/真实 HuNav 服务测试。
4. `a048bfd` **测试与交付**：受控矩阵驱动器、final run logging、patch、回归记录、
   HANDOFF、本文及外部文档同步。
5. **视觉一致性修复**：Regular 单目标处残留运动归零、可视化驱动器、Character 四元数
   Surprised 朝向 gate、相关单元/服务/GPU 回归；其 commit 无法自引用，使用
   `git rev-parse HEAD` 获取。
6. **Character 坐标帧修复**：ROS `+X` 与 Isaac People `-Y` 前向轴双向转换、Scared
   danger/lost 安全优先、三模式 GPU 复核、补丁与文档刷新；同样以最终 HEAD 获取 commit。
7. **活动源码交付收口**：原工作区定向备份/合并、独立 `.colcon-formal-v1` 构建测试、
   sudden 与 strict 实测、无根 Git manifest 兼容、测试 base-path 隔离及本文第 16 节证据；
   该提交同样无法在自身正文中写入完整 hash，以最终 `git rev-parse HEAD` 为准。

bridge 变化后，基于 `upstream/manifest.tsv` 中固定 arena-isaac commit
`16b8e3416517d8c3dc1b5038df4fe11b9a6df46c` 重新生成二进制安全
`patches/arena-isaac.patch`。在干净临时上游树验证：

- 正向 `git apply --check` 成功；
- 应用后反向 `git apply --reverse --check` 成功；
- patch 包含原归档改动、bridge 改动、Character 坐标转换及其纯测试，没有活动工作区杂项。

补丁 SHA-256 为
`1d404fca247c81a6dfbfaa470cfd35a7ca8005d0b2cd2be056fd9bf141875b23`。它已经在
`/tmp/social-nav-x-arena-isaac-verify-character-frame.7Xjf7m` 正向检查、实际应用、应用后
反向检查，并把由 patch numstat 枚举的 31 个文件逐一与 feature tree 比对一致。补丁生成树
为 `/tmp/social-nav-x-arena-isaac-refresh.E29zGd`。不要把 feature 自动合并到 `main` 或推送
远端。

## 14. 最终交付与复现命令

`HANDOFF.md` 已新增独立 formal automata 章节，记录 feature/baseline、命令、哈希、证据、
回退和单人限制。最小复现命令：

```bash
cd /home/lpc/workspace/social-nav-x-formal-v1
scripts/build_formal_overlay.sh
scripts/test_formal_overlay.sh

# 单次 production-style demo（默认 GPU 3、NAVIGATION=false、ideal chassis）
scripts/run_formal_social_demo.sh

# 第二终端：通过 WebRTC 观察并同时获取定量状态/角度输出；三种场景每次选一个
scripts/run_formal_social_visual_scenario.sh safe
scripts/run_formal_social_visual_scenario.sh sudden 2.0
scripts/run_formal_social_visual_scenario.sh fast 2.0

# 无 WebRTC 的 production-style headless 运行
env DRL_VO_GUI=false scripts/run_formal_social_demo.sh \
  headless:=true livestream:=false foxglove:=false

# 两轮三场景完整验收；domain base 可在无冲突的 [0,226] 中选择
env DRL_VO_GUI=false GPU_ID=3 FORMAL_ACCEPTANCE_DOMAIN_BASE=181 \
  scripts/test_formal_simulation_matrix.sh
```

原工作区源码合并后的隔离构建与可视化命令如下；不得改用共享 `build/install/log`：

```bash
cd /home/lpc/workspace/arena5_ws
env ARENA_BASE_WS="$PWD" FORMAL_OVERLAY_ROOT="$PWD/.colcon-formal-v1" \
  scripts/build_formal_overlay.sh
env ARENA_BASE_WS="$PWD" FORMAL_OVERLAY_ROOT="$PWD/.colcon-formal-v1" \
  scripts/test_formal_overlay.sh

# 终端 1
GPU_ID=3 NAVIGATION=false FORMAL_OVERLAY_ROOT="$PWD/.colcon-formal-v1" \
  scripts/run_formal_social_demo.sh
# 终端 2：三者每次只运行一个
FORMAL_OVERLAY_ROOT="$PWD/.colcon-formal-v1" \
  scripts/run_formal_social_visual_scenario.sh sudden 2.0
```

生效配置 SHA-256：

```text
formal_social_automata.yaml  5a8be544cabfc0d7f6546c496d22f4ec521fc366d578578f36267b4d53857693
formal_social_agent.yaml     a3f03ea3ea74178377e06aea86e764f47225e3daeb90abd219080257f78b47b6
```

交付内容包括：

- feature 分支最终 commit 和基线 commit/tag；
- overlay 的准确 build/source 命令和 formal demo 启动命令；
- 生效配置文件及 SHA-256、运行日志/transition trace 的绝对位置；
- 单元、服务、仿真、原六行为/Nav2 回归的精简结果和 marker；
- 未运行或失败项目及复现信息；
- 快速回退与完整恢复链接；
- HuNav reset 带来的最多一拍可见切换延迟；
- V1 只支持单人、多人前必须重新设计按-agent 动态换树的限制。

本文完成后通过字节级 `cmp` 同步到用户指定的外部路径
`/home/lpc/social-nav-x_formal_social_automata_development_spec.md`。仓库中的原始快照始终
保持不变，并继续以 `e0333348...a268` 追溯最初附件。

## 15. 后续 Codex Agent 执行检查单

1. 先读 `BASELINE.md`、本文、`HANDOFF.md` 和恢复说明。新开发只在 feature worktree；若
   必须维护已合并活动源码，先核对第 16 节 allowlist、定向快照和嵌套仓库现有状态。
2. 检查 `git status`，区分其他 Agent/用户已有改动，禁止覆盖或清理。
3. 修改前保存相关文件哈希；只在当前阶段范围内编辑。
4. 核心先纯测试，再接 ROS；所有状态变更遵循 candidate/commit。
5. 选择性构建独立 overlay，不安装依赖，不运行活动工作区全量 build。
6. formal demo 使用独立入口；快速接近关闭 Nav2 并确认单一 `/cmd_vel` publisher。
7. 记录真实测试证据，刷新 arena-isaac patch 并验证正反 apply。
8. 对照守护哈希和原 marker 回归；任何基线变化先停止交付并调查。
9. 更新 HANDOFF 和本文，仅把已取得的结果写成“通过”。

## 16. 原工作区源码合并与坐标问题结论（2026-09-02）

### 16.1 是否存在同一 `90°` 问题

结论是**存在且需要修复**。合并前实际运行副本
`src/arena-isaac/arena_isaac/pedestrian/simulator/logic/people/person.py` 的 SHA-256 为
`883c1aaec2c242521015313c57f8ce562d5a6122ada27b6c4bcb6a58615df684`；它把 ROS
四元数直接交给 Character Graph，并直接读回 graph 四元数，没有资产前向轴修正。
`character_frames.py` 当时不存在。由于 ROS 平面 yaw 以局部 `+X` 为正前方，而当前 Isaac
People 资产视觉正前方为局部 `-Y`，同一个数值 yaw 会在画面中相差约 `90°`。

修复只发生在 ROS/Character 边界：

```text
写入 Character：q_character = q_ros * qz(+pi/2)
读回 ROS：      q_ros       = q_character * qz(-pi/2)
```

因此 `/human_states`、HuNav、自动机 FOV 和 verifier 继续使用 ROS `+X` 语义；仅 Isaac
资产得到所需偏移。合并后活动副本与 feature 字节一致：

```text
Person.py            429a25528bb9cc5cd7b16616a6099d0644777e90155e501ff0099c544adfa7f1
character_frames.py  2e018db9ad9c34c8e4cedb057637628d08f6c7dd4be0fe74fe5ac5cf47d01eaa
test_character_frames.py
                     53d577dcd224ed2b5295ed001dc2e8f208c926ace2436158c03771701a315233
```

该边界转换同样服务于 Curious、Surprised、Scared 的显示姿态，不改变它们的 HuNav 速度、
目标或状态机转移。原工作区 sudden 实测停止速度 `0.000000 m/s`，面对机器人误差
`2.883°`；原六行为 strict marker 仍为 types `1,2,3,4,5,6`，说明转换没有破坏现有
行为入口。

### 16.2 合并、构建与验证

活动根目录不是 Git 仓库，所以没有伪造顶层 commit。合并内容由关键源码组合 SHA-256
`3ac7a64e30f381221cc0835059ac61c43e86961f603e582625bf58d529e7e2f4` 标识；run script
在此布局下写入 `source_revision_kind=merged_content_sha256`，不再打印根 Git 错误。

初始源码合并只同步：formal 包与其配置/launch/test、formal 构建/测试/demo 脚本与文档、bridge
通用模式及测试、`Person.py`、`character_frames.py` 和其纯测试。原六行为脚本/YAML/launch、
D6、碰撞、Nav2、共享 install、Conda 环境、lockfile、`.repos` 和 manifest 当时没有修改；
第 17 节的后续修复只修改六行为入口脚本，不改 YAML/launch 或依赖。11 个
嵌套仓库的 Git index 哈希在复制、构建和测试前后完全一致。

独立输出根为 `/home/lpc/workspace/arena5_ws/.colcon-formal-v1`。只构建
`arena_isaac`、`arena_humble_compat`、`formal_social_behavior`，三者 package prefix 均
解析到该 overlay。测试结果为 frame `17/17`、compat `20/20`、formal `76/76`；xUnit 为
compat `20`、formal `261`，均 `0 errors, 0 failures, 0 skipped`。测试脚本显式限制三个
`--base-paths`，避免 colcon 扫描活动工作区中未由该 overlay 构建的 Nav2 包。

运行证据：

```text
/home/lpc/workspace/arena5_ws/logs/formal_visual/20260902_110743_494419251_sudden_pid2323558/visual.log
SHA-256 da860f67cb99d76bafc9977842c68491ecad21939fa1c8927838241775fb1e3b
FORMAL_SOCIAL_SCENARIO_OK ... target=SURPRISED ... surprised_speed=0.000000 ... surprised_facing_error_deg=2.883

/home/lpc/workspace/arena5_ws/logs/regression/formal_source_merge_20260902/verify_six_behaviors.log
SHA-256 ad0c1a66232befc049c484d818653db68666363a8ce5c5bdc88490229bf235cf
SIX_BEHAVIORS_VERIFY_OK types=1,2,3,4,5,6 active=3,5 responses=3,4,5,6 robot_distance=0.863 robot_states=434
```

### 16.3 版本保护与未执行测试

定向恢复包位于
`/home/lpc/workspace/arena5_ws_archives/20260902_formal_source_merge_pre/`，恢复步骤以其中
`RESTORE.md` 为准。`pre_existing_targets.tar.zst` SHA-256 为
`024672330865d7500ee2af9045e0d976bb9dc5f3f8f398ddb96dcb24b9fa6e27`，已通过 zstd 和
tar 列表校验；恢复时先把新增路径移动到保留目录，再解压旧文件，禁止覆盖式回滚。

按用户 2026-09-02 的明确要求，底盘和碰撞后续项已停止：

- 首次底盘 case 1 遇到 `/SpawnUrdf` 启动时序问题，未生成有效测量；日志保留在
  `logs/chassis_control/20260902_formal_source_merge_matrix/`。
- 重试的 `results.csv/results.json` 落盘 `18/22` 个有效 case，18 个均 `valid=true`；
  `results.csv` SHA-256 为
  `9fc5fc87384de90e11db55965fd213806773ee51298f1a18b9853e890acb4d4a`，`results.json`
  为 `87a093b8e354b61f934e6a89770be9b70a921ee26e7f6ca5d282a01ede484189`。
- 剩余底盘 `4/22` 未执行，碰撞 `0/3` 未执行；不得写成通过。对应进程已停止，不会在
  后台自动续跑。

这份规范有意将自动机、HuNav profile、ROS transaction 和 Isaac 显示分层。V1 的研究
增量是可重放、可检查的离散社会状态；现有 HuNav/SFM、D6、Nav2 和 Character 仍保持
各自 authority，不因引入形式化层而被替换。

## 17. 主六行为入口坐标修复落地（2026-09-02）

### 17.1 再次出现 `90°` 偏差的实际根因

活动源码及隔离 overlay 已包含第 16 节的双向 Character 坐标转换，但主入口
`scripts/run_six_behaviors.sh` 当时只执行 `scripts/env.sh`，因此运行时解析到旧的共享
`install/arena_isaac`。两个安装层的实际 `Person.py` SHA-256 为：

```text
共享 install（旧）  883c1aaec2c242521015313c57f8ce562d5a6122ada27b6c4bcb6a58615df684
隔离 overlay（新） 429a25528bb9cc5cd7b16616a6099d0644777e90155e501ff0099c544adfa7f1
活动源码（新）      429a25528bb9cc5cd7b16616a6099d0644777e90155e501ff0099c544adfa7f1
```

因此 formal demo 正常而主六行为仍偏转并不是转换公式失效，而是两个入口选择了不同安装
层。源码合并、隔离构建和运行时选择必须三者同时成立。

### 17.2 主入口修复（中间态，已由第 18 节取代）

主入口现在默认设置 `ARENA_SIX_BEHAVIORS_USE_OVERLAY=true`，在共享 underlay 之后加载
`/home/lpc/workspace/arena5_ws/.colcon-formal-v1/install/local_setup.bash`，并在启动 Isaac 前：

1. 检查 `arena_isaac` 和 `arena_humble_compat` 的 prefix 必须位于该 overlay；
2. 检查安装后的 `character_frames.py` 存在；
3. 对活动源码与 overlay 中的 `Person.py` 做字节级比较；
4. 将 prefix、`Person.py` 哈希和 `ros_plus_x_to_isaac_minus_y` 写入每次运行的
   `runtime_manifest.txt`；
5. overlay 缺失或过期时直接失败并给出隔离重建命令，不回退到旧共享安装。

该入口修复提交为 `0359579`。

正常启动命令不变，也不需要用户手工 source overlay：

```bash
cd /home/lpc/workspace/arena5_ws
GPU_ID=3 ./scripts/run_six_behaviors.sh

# 不启动 Isaac 的快速检查
GPU_ID=3 ./scripts/run_six_behaviors.sh --check-overlay-only
```

`ARENA_SIX_BEHAVIORS_USE_OVERLAY=false` 只保留作诊断或紧急回退，它会选择旧共享安装并可能
重现视觉偏差，不是正常验收模式。共享 `install` 仍未重建或修改，符合“源码合并、构建隔离”
约束。

### 17.3 对六种行为的影响

六种行为最终都通过同一个 `Person`/Character Graph 边界渲染，转换与 behavior type 无关：

| 行为 | 修复后的显示影响 | 保持不变的 HuNav 语义 |
|---|---|---|
| Regular | 行走身体前向与 ROS 轨迹一致 | 常规目标与速度 |
| Impassive | 行走身体前向与 ROS 轨迹一致 | 无社会响应的常规导航 |
| Surprised | 静止时按命令 yaw 面向机器人，不再偏 `90°` | 停止、计时与状态 |
| Scared | 逃离时身体方向与离开速度一致 | 逃离目标、速度和力 |
| Curious | 接近时身体方向与接近速度一致 | 接近距离、速度和计时 |
| Threatening | 逼近目标时身体方向一致 | 目标点、速度和计时 |

写入 Character 使用 `q_character = q_ros * qz(+pi/2)`，反馈使用
`q_ros = q_character * qz(-pi/2)`；所以 `/human_states`、HuNav FOV、速度、目标、力系数、
behavior type 和自动机转移均保持 ROS `+X` 语义。无需也没有对六套行为分别增加补偿。

### 17.4 验证结果与证据

`--check-overlay-only` 得到修复后的两个 overlay prefix 和
`person_sha256=429a2552...fa7f1`。GPU 3 主入口的实际 Isaac 进程为：

```text
/home/lpc/workspace/arena5_ws/.colcon-formal-v1/install/arena_isaac/lib/arena_isaac/run_isaacsim
```

真实 HuNav/Isaac 六行为回归通过：

```text
SIX_BEHAVIORS_VERIFY_OK types=1,2,3,4,5,6 active=3,5 responses=3,4,5,6 robot_distance=1.020 robot_states=456
```

Character 坐标纯测试为 `17/17`；稳定运行段 compute 为 `14.848--18.399 Hz`、display 为
`4.806--4.891 Hz`、`max_dt=0.025 s`。运行在验证后经 `Ctrl-C` 正常停止。证据为：

```text
/home/lpc/workspace/arena5_ws/logs/regression/six_behavior_overlay_heading_fix_20260902/verification.txt
SHA-256 32b3edcd9b1edfb5208c6f160db202f3efa386f69fa7e553cabc6581b34f691a
/home/lpc/workspace/arena5_ws/logs/runs/20260902_163617_six_behaviors_gpu3/runtime_manifest.txt
SHA-256 6bbe7ad11815d900251568b4155aef509b04d134b24fe07f615031d690e8db29
```

### 17.5 版本保护与恢复

修改前的入口保存在
`/home/lpc/workspace/arena5_ws_archives/20260902_six_behavior_overlay_entry_pre/`：

```text
旧 run_six_behaviors.sh  bd7459f3dc75cf770cc9985a1d6c5bb7c3aea2c54ddce58b6bd312c2a74077a2
RESTORE.md               2461d12027ee7b8491417fd34af403368cf7a162e1272f41d04ec7d387e9bc99
新 run_six_behaviors.sh  24abb1ee73e2ca66aad1c352570c2e1d757fcfadb7ee9f1617c768f54d7f9633
```

恢复时按该 `RESTORE.md` 先保留当前脚本再复制旧脚本，禁止对任何嵌套仓库执行
`reset/clean/checkout`。本次未修改共享 install、Conda 元数据、依赖、六行为 YAML/launch、
D6、Nav2 或碰撞文件；嵌套仓库 index 哈希保持一致。按用户要求，本轮继续不运行底盘矩阵和
碰撞套件，不能将它们写成通过。

## 18. 主工作区共享 install 部署（2026-09-02）

本节是第 17 节 overlay 中间方案之后的权威最新状态。用户明确要求将主工作区源码编译到
共享 `install`，因此本轮允许对 `build/arena_isaac` 和 `install/arena_isaac` 做一次定向、
可恢复的选择性构建；这项授权不扩展到全量 build、其他共享包或依赖安装。

### 18.1 实际源码清单

坐标修复源码已经在第 16 节合并到活动工作区，本轮先确认内容正确，未重复叠加另一个角度
补偿。权威源码是：

1. `src/arena-isaac/arena_isaac/pedestrian/simulator/logic/people/character_frames.py`
   - 新增不依赖 Isaac/rclpy 的四元数归一化与乘法；
   - `ros_to_character_quaternion()` 实现 `q_ros * qz(+pi/2)`；
   - `character_to_ros_quaternion()` 实现 `q_character * qz(-pi/2)`。
2. `src/arena-isaac/arena_isaac/pedestrian/simulator/logic/people/person.py`
   - 在 Character Graph 写入、初始 spawn 和 `set_world_pose()` 边界应用正向转换；
   - `update_state()` 读回 graph 姿态时应用逆向转换，保持 ROS `/human_states` 语义；
   - 保存并归一化 HuNav 命令姿态，静止时让 Surprised 使用显式 look-at yaw；
   - 行走时继续由 locomotion/PathPoints 控制方向，避免 Curious、Scared 等移动行为被静止
     姿态覆盖。
3. `src/arena-isaac/arena_isaac/test/test_character_frames.py`
   - 覆盖七个 yaw 的局部 `-Y` 视觉前向、五个往返 yaw、`+90°` 常量和非法四元数，共
     `17` 个测试。
4. `scripts/run_six_behaviors.sh`
   - 最新提交 `d8b026d` 默认使用共享 install；
   - 启动前验证 `arena_isaac`/compat prefix、`character_frames.py` 存在及
     `Person.py` 与活动源码字节一致；
   - `--check-runtime-only` 提供无 Isaac 的快速校验；
   - `ARENA_SIX_BEHAVIORS_USE_OVERLAY=true` 仅保留作 overlay 对照。

源码哈希为：

```text
Person.py                   429a25528bb9cc5cd7b16616a6099d0644777e90155e501ff0099c544adfa7f1
character_frames.py         2e018db9ad9c34c8e4cedb057637628d08f6c7dd4be0fe74fe5ac5cf47d01eaa
test_character_frames.py    53d577dcd224ed2b5295ed001dc2e8f208c926ace2436158c03771701a315233
run_six_behaviors.sh         9f8d26eae21348e00e6141f95829e51bf5cbc21e0c57d541bd3c8e9df2530836
```

### 18.2 共享构建与安装结果

构建前保存了 `build/arena_isaac`、`install/arena_isaac`、入口脚本和上述三份坐标源码。
随后只执行：

```bash
cd /home/lpc/workspace/arena5_ws
source scripts/env.sh
colcon build --event-handlers console_cohesion+ \
  --packages-select arena_isaac \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

结果为 `Finished <<< arena_isaac [1min 30s]`、`1 package finished [1min 31s]`，构建日志位于
`/home/lpc/workspace/arena5_ws/log/build_2026-09-02_16-57-32`。没有运行全量
`scripts/build.sh`，没有构建 formal、Nav2、D6、碰撞、HuNav 或依赖包。

将构建前归档解到临时目录并对共享 `install/arena_isaac` 做递归比较后，运行源码差异只有：

```text
替换  pedestrian/simulator/logic/people/person.py
新增  pedestrian/simulator/logic/people/character_frames.py
刷新  egg-info/SOURCES.txt 和对应 Python 字节码
```

Colcon 同时刷新了顶层生成文件 `install/setup*`、`local_setup*` 的时间戳；包清单没有增删，
`install/setup.bash` 的 SHA-256 仍为
`e3b0addf5e333f92b50598538d132869b8ee08bcad6cc4ef3e9361fdfaada898`。

安装后的两份源码与活动源码 `cmp` 均为 `0`，哈希分别为 `429a2552...fa7f1` 与
`2e018db9...1eaa`。这证明运行 install 不是旧缓存，也没有意外覆盖其他包。

### 18.3 共享 install 验证

无启动预检：

```text
SIX_BEHAVIORS_RUNTIME_OK mode=shared arena_isaac=/home/lpc/workspace/arena5_ws/install/arena_isaac compat=/home/lpc/workspace/arena5_ws/install/arena_humble_compat person_sha256=429a2552...fa7f1
```

使用共享安装路径作为 `PYTHONPATH` 的坐标测试为 `17 passed in 0.02s`。随后 GPU 3 主入口
实测确认进程为：

```text
/home/lpc/workspace/arena5_ws/install/arena_isaac/lib/arena_isaac/run_isaacsim
```

真实六行为验证通过：

```text
SIX_BEHAVIORS_VERIFY_OK types=1,2,3,4,5,6 active=3,5 responses=3,4,5,6 robot_distance=0.967 robot_states=372
```

稳定样本为 compute `14.6--15.1 Hz`、display `4.8 Hz`、`max_dt=0.025 s`。运行完成后经
`Ctrl-C` 正常停止，无遗留 Isaac、HuNav 或 Foxglove 进程。证据：

```text
/home/lpc/workspace/arena5_ws/logs/regression/shared_install_character_frame_fix_20260902/verification.txt
SHA-256 72ef2199f4cc5fa56f9d5c84ab7d35e58c97a6a44adc206c098ef30dc2b39596
/home/lpc/workspace/arena5_ws/logs/runs/20260902_170015_six_behaviors_gpu3/runtime_manifest.txt
SHA-256 d357874656afa1beee2737890702ae5ad129f56c0921faee66b52ef6451d5b0c
```

### 18.4 恢复与依赖保护

定向恢复目录为
`/home/lpc/workspace/arena5_ws_archives/20260902_shared_arena_isaac_install_pre/`：

```text
pre_deploy_targets.tar.zst  6905ce412deb772da3c04819f6942558707b0bcb98a21996c9cddb8070466f4d
RESTORE.md                   a95b4ddba756274c387249bc2ca9b3599d5e7ce0917cd4680d3a5434f6560b6e
NESTED_GIT_INDEX_SHA256      243d2103a6db2207ca291f355756596806b4c84654e48dc5dfd5296e23d2205b
```

压缩包通过 `zstd -t` 和 `tar --zstd -tf`。恢复流程先移动保存当前 build/install/源码/脚本，
再解压旧目标，禁止覆盖式恢复和 Git reset/clean。Conda history、依赖、其他共享 install 包、
六行为 YAML/launch、Nav2、D6 和碰撞文件均未修改；11 个嵌套仓库 index 哈希保持不变。
底盘矩阵和碰撞测试继续按用户此前要求跳过，未将其标记为通过。

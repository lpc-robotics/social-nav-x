# social-nav-x：形式化人群社交行为自动机开发说明

> 目标读者：Codex Agent / 开发人员  
> 项目仓库：https://github.com/lpc-robotics/social-nav-x.git  
> 目标工作区：`~/workspace/arena5_ws`  
> 当前平台：Ubuntu 22.04 + ROS 2 Humble + Isaac Sim 5.1 + Arena-Rosnav 5.0 + HuNavSim v1  
> 目标：在不破坏当前已验证仿真链路的前提下，引入“形式化人群社交行为自动机”，使行人能够根据机器人和其他行人的行为动态改变社交/心理状态，并驱动 HuNav 的连续运动行为。

---

## 1. 当前项目状态

当前仓库已经完成并验证以下链路：

```text
Isaac Sim 5.1
  ├─ Jackal
  ├─ PhysX collision
  ├─ odom / TF / lidar / point cloud
  ├─ WebRTC / Foxglove
  ├─ Nav2
  └─ HuNav 六种固定行为
       ├─ Regular
       ├─ Impassive
       ├─ Surprised
       ├─ Scared
       ├─ Curious
       └─ Threatening
```

当前六行为链路：

```text
HuNav YAML
  -> hunav_loader
  -> hunav_agent_manager /compute_agents
  -> arena_humble_compat/hunav_six_behaviors_bridge
  -> Arena Isaac pedestrian services
  -> Isaac Character
```

机器人状态通过 `/odom` 送入 HuNav；HuNav 负责社会行为和 SFM，Isaac 只负责人物显示、动画和物理环境。

当前已经通过：

- Jackal 速度测试 22/22；
- 碰撞测试 3/3；
- Nav2 smoke test；
- `SIX_BEHAVIORS_VERIFY_OK types=1,2,3,4,5,6`；
- 长时间运行稳定性验证。

因此当前项目应视为**稳定 Baseline**。后续开发不能破坏现有六行为 demo、D6 底盘、Nav2 和 Isaac Character 链路。

---

## 2. 当前模型的不足

当前六行为是“固定人物类型”：

```text
Person 1 永远 Regular
Person 2 永远 Impassive
Person 3 永远 Surprised
Person 4 永远 Scared
Person 5 永远 Curious
Person 6 永远 Threatening
```

这适合做行为验证，但不是真正的动态社会心理模型。

目标是改成：

```text
同一个 Human
    |
    | RobotVisible
    v
 Attention
   /   \
  /     \
TTC低   安全接近
 |        |
 v        v
Scared  Curious
   \      /
    \    /
   RobotLeaving
       |
       v
     Normal
```

即：**行为类型不再是人物的永久属性，而是由当前环境事件驱动的离散状态。**

---

## 3. 总体研究思想

### 3.1 每个行人建立一个离散自动机

定义：

\[
H_i=(Q_i,\Sigma_i,\delta_i,q_{i,0})
\]

其中：

- \(Q_i\)：社会/心理状态集合；
- \(\Sigma_i\)：机器人或其他人触发的事件；
- \(\delta_i\)：状态转移函数；
- \(q_{i,0}\)：初始状态。

第一版建议状态：

```text
NORMAL
ATTENTION
CURIOUS
SURPRISED
SCARED
THREATENING
SOCIAL
```

注意：这里的状态是“离散社会行为/心理状态”，不要与连续位置速度混在一起。

### 3.2 连续运动状态

每个行人仍有：

\[
x_i=[x_i,y_i,v_{x,i},v_{y,i}]
\]

HuNav/SFM 继续负责连续运动。

因此整体属于：

\[
\boxed{\text{Hybrid System}}
\]

即：

```text
离散状态 q_i
    ↓
选择行为模式/参数
    ↓
HuNav / SFM
    ↓
连续状态 x_i
```

### 3.3 机器人必须参与状态转移

机器人不能只作为 obstacle。

事件必须包含机器人对人的刺激，例如：

```text
ROBOT_VISIBLE
ROBOT_NEAR
ROBOT_FAST_APPROACH
TTC_LOW
PERSONAL_SPACE_VIOLATION
ROBOT_YIELDING
ROBOT_LEAVING
```

因此形成闭环：

```text
Robot action
   ↓
Human perception/event
   ↓
Human automaton
   ↓
Human behavior
   ↓
Human trajectory
   ↓
Robot observation
   ↓
Social navigation planner
   ↓
Robot action
```

### 3.4 多人交互

以后增加：

```text
PEER_VISIBLE
MUTUAL_GAZE
PEER_NEAR
GROUP_FORMED
GROUP_BROKEN
```

对于两个行人：

\[
H_A \parallel H_B
\]

不要求在运行时代码中真的构造指数规模的笛卡尔积状态空间，而是：

- 每个 agent 保持独立自动机；
- 通过共享事件进行同步；
- 离线形式化分析时再构造组合模型。

这样避免状态爆炸。

---

## 4. 第一版自动机规范

### 4.1 建议状态

#### NORMAL

普通行走状态，对应 HuNav `Regular`。

#### ATTENTION

检测到机器人后进入观察状态。该状态可作为短暂过渡状态，不直接对应最终 HuNav 行为。

#### CURIOUS

机器人可见、危险程度低、靠近方式平稳时，行人主动接近机器人。

对应 HuNav `Curious`。

#### SURPRISED

机器人突然进入较近区域、但未形成强烈碰撞风险时，行人停止并观察机器人。

对应 HuNav `Surprised`。

#### SCARED

机器人高速接近、TTC 很低或侵犯个人空间时，行人主动远离机器人。

对应 HuNav `Scared`。

#### THREATENING

保留现有 HuNav threatening 行为作为特殊可配置状态。第一版不要求从真实心理模型推导，可作为测试状态或显式角色配置。

#### SOCIAL

预留给人-人交互，例如 conversation / group / following。第一版可只实现状态和事件，不必立即实现复杂行为。

---

## 5. 第一版事件规范

不要在自动机内部直接读取 ROS topic。必须先做 Event Extractor，把连续变量转成离散事件。

建议输入：

```text
robot pose
robot velocity
human pose
human velocity
human yaw
peer poses
peer velocities
sim time
```

建议计算：

```text
robot_distance
relative_velocity
bearing_to_robot
robot_visible
robot_approaching
TTC
personal_space_violation
robot_leaving
peer_distance
mutual_gaze
```

建议事件：

```text
ROBOT_VISIBLE
ROBOT_LOST
ROBOT_NEAR
ROBOT_FAR
ROBOT_FAST_APPROACH
TTC_LOW
PERSONAL_SPACE_VIOLATION
ROBOT_LEAVING
TIMEOUT
PEER_VISIBLE
PEER_NEAR
MUTUAL_GAZE
```

第一版阈值必须放配置文件，不要硬编码。

例如：

```yaml
formal_social_behavior:
  robot_visible_distance: 6.0
  robot_near_distance: 2.5
  personal_space_distance: 1.2
  ttc_low: 1.5
  fast_approach_speed: 0.8
  attention_timeout: 2.0
  recovery_timeout: 3.0
```

必须增加 hysteresis / cooldown，避免距离在阈值附近产生状态抖动。

---

## 6. 第一版状态转移建议

建议初始实现：

```text
NORMAL
  --ROBOT_VISIBLE--> ATTENTION

ATTENTION
  --TTC_LOW----------------------> SCARED
  --PERSONAL_SPACE_VIOLATION-----> SCARED
  --ROBOT_FAST_APPROACH----------> SCARED
  --ROBOT_NEAR + safe approach---> CURIOUS
  --TIMEOUT----------------------> SURPRISED
  --ROBOT_LOST-------------------> NORMAL

CURIOUS
  --TTC_LOW----------------------> SCARED
  --ROBOT_LEAVING----------------> NORMAL
  --ROBOT_LOST-------------------> NORMAL

SURPRISED
  --TTC_LOW----------------------> SCARED
  --ROBOT_LEAVING----------------> NORMAL
  --recovery timeout-------------> NORMAL

SCARED
  --ROBOT_LEAVING + safe distance--> NORMAL

NORMAL / ATTENTION
  --explicit threatening role-----> THREATENING
```

第一版要求**确定性**：

> 对任意状态 + 当前事件集合，必须能唯一决定下一状态。

若多个事件同时触发，需要定义优先级，例如：

```text
PERSONAL_SPACE_VIOLATION
> TTC_LOW
> ROBOT_FAST_APPROACH
> ROBOT_NEAR
> ROBOT_VISIBLE
> TIMEOUT
```

---

## 7. HuNav 行为映射

自动机不直接写人物位置。

自动机只输出：

```text
social_state
behavior_mode
optional behavior parameters
```

建议初始映射：

| Automaton State | HuNav Mode |
|---|---|
| NORMAL | Regular |
| ATTENTION | Regular 或保持当前行为 |
| CURIOUS | Curious |
| SURPRISED | Surprised |
| SCARED | Scared |
| THREATENING | Threatening |
| SOCIAL | 第一版先保持 Regular |

### 重要要求

必须首先验证：

> HuNav v1 是否支持运行过程中修改 `agent.behavior.type` 并立即切换行为。

如果支持，则直接使用 runtime behavior switching。

如果不支持，则不得强行修改 HuNav 核心逻辑。应先分析：

- 是否可以重置 `behavior.state`；
- 是否可以动态修改 `dist / vel / force_factor / goal`；
- 是否需要在 bridge 层实现 behavior adapter。

这一点必须先做最小实验，不要直接大规模重构。

---

## 8. 推荐软件结构

优先新建独立 ROS 2 Python package，避免把所有逻辑塞进 bridge。

建议：

```text
src/arena-isaac/formal_social_behavior/
├── package.xml
├── setup.py
├── resource/
├── formal_social_behavior/
│   ├── __init__.py
│   ├── model.py
│   ├── event_extractor.py
│   ├── automaton.py
│   ├── manager.py
│   ├── behavior_adapter.py
│   ├── trace_logger.py
│   └── config.py
├── config/
│   └── automata.yaml
└── test/
    ├── test_event_extractor.py
    ├── test_automaton.py
    ├── test_hysteresis.py
    └── test_parallel_agents.py
```

如果新 package 会显著增加编译复杂度，可以先放在：

```text
src/arena-isaac/arena_humble_compat/arena_humble_compat/formal_social_behavior/
```

但长期建议独立 package。

---

## 9. 与当前 bridge 的集成点

当前关键文件：

```text
src/arena-isaac/arena_humble_compat/
  arena_humble_compat/hunav_six_behaviors_bridge.py
```

当前每次 HuNav 更新前已有：

```python
request = ComputeAgents.Request()
request.current_agents = copy.deepcopy(self._agents)
request.robot = self._robot_agent()
self._compute_future = self._compute.call_async(request)
```

建议改为：

```python
robot = self._robot_agent()

social_update = self._formal_manager.update(
    agents=self._agents,
    robot=robot,
    stamp=stamp,
)

agents_for_hunav = self._behavior_adapter.apply(
    agents=self._agents,
    social_update=social_update,
)

request = ComputeAgents.Request()
request.current_agents = copy.deepcopy(agents_for_hunav)
request.robot = robot

self._compute_future = self._compute.call_async(request)
```

必须保证：

- Isaac Character 不新增社会行为逻辑；
- 不直接写人物 pose；
- HuNav 继续作为连续运动 authority；
- bridge 仍然只负责状态传递和适配。

---

## 10. Feature Flag

所有新功能必须可以关闭。

建议：

```text
FORMAL_SOCIAL_AUTOMATA=false
```

或 ROS parameter：

```yaml
formal_social_automata:
  enabled: false
```

要求：

### enabled=false

行为必须与当前 Baseline 一致。

### enabled=true

启用动态自动机。

现有六行为 demo 不能被直接覆盖。建议额外新建：

```text
run_formal_social_demo.sh
formal_social_demo.launch.py
formal_social_agents.yaml
```

不要把原有 `run_six_behaviors.sh` 改成新的实验入口。

---

## 11. 开发阶段

### Phase 0：冻结 Baseline

先确认并记录：

```bash
source scripts/env.sh
GPU_ID=3 ./scripts/run_six_behaviors.sh
```

并保留以下验证结果：

```text
SIX_BEHAVIORS_VERIFY_OK
SMOKE_NAVIGATION_OK
```

禁止：

- 升级 HuNavSim v2；
- 修改 D6 chassis；
- 修改 Isaac `Person.py` 社会行为；
- 重构 Nav2；
- 修改已有六行为 YAML 的语义。

---

### Phase 1：只实现 Event Extractor

暂时不改变人物行为。

要求：

- 从 robot + human 状态计算 distance / bearing / TTC；
- 生成离散事件；
- 发布 debug 日志；
- 单元测试；
- 阈值配置化；
- hysteresis 测试。

建议日志：

```text
agent=4
state=NORMAL
events=[ROBOT_VISIBLE, ROBOT_NEAR]
distance=2.12
ttc=3.84
```

---

### Phase 2：1 Robot + 1 Human 自动机

先做单人，不做多人。

目标状态：

```text
NORMAL
ATTENTION
CURIOUS
SURPRISED
SCARED
```

要求：

- 自动机与 HuNav 解耦；
- 纯 Python 单元测试可以直接驱动事件；
- 记录 transition trace：

```text
time
agent_id
old_state
event
new_state
```

示例：

```text
12.30  human_1  NORMAL     ROBOT_VISIBLE  ATTENTION
13.10  human_1  ATTENTION  TTC_LOW        SCARED
16.80  human_1  SCARED     ROBOT_LEAVING  NORMAL
```

---

### Phase 3：自动机驱动 HuNav 行为

先验证 runtime behavior switching。

测试至少覆盖：

```text
NORMAL -> CURIOUS -> NORMAL
NORMAL -> SURPRISED -> NORMAL
NORMAL -> SCARED -> NORMAL
ATTENTION -> SCARED
CURIOUS -> SCARED
```

要求：

- 人物运动由 HuNav 计算；
- 不直接写 pose；
- 状态切换后 Isaac Character 动画仍正常；
- HuNav compute wall-clock 频率不能明显跌破当前可用范围；
- 无明显状态抖动。

---

### Phase 4：1 Robot + 2 Humans 并行自动机

增加：

```text
H_A || H_B
```

但运行时采用本地自动机 + 共享事件。

新增事件：

```text
PEER_VISIBLE
PEER_NEAR
MUTUAL_GAZE
```

第一版多人行为只需实现一个简单场景：

```text
A sees B
B sees A
both close enough
-> MUTUAL_GAZE
-> both enter SOCIAL
```

机器人进入二者之间时：

```text
ROBOT_NEAR / TTC_LOW
-> interaction broken
-> individual automata transition
```

不要在第一版加入复杂 group formation / follow / conversation animation。

---

### Phase 5：形式化验证接口

目标不是马上做完整理论论文工具，而是让运行时模型可导出、可验证。

至少实现：

- 状态集合导出；
- 转移表导出；
- 不可达状态检查；
- 非确定性转移检查；
- dead-end state 检查。

后续可增加 UPPAAL 模型。

建议验证性质：

```text
1. SURPRISED 不是永久状态
2. SCARED 在机器人离开后最终恢复
3. 同一个事件集合不能产生两个不同下一状态
4. 没有未定义的状态
5. 没有无出口的瞬态状态
```

未来 UPPAAL 可验证：

```text
A[] not deadlock
A[] (state == SURPRISED imply eventually NORMAL)
A[] (TTC_LOW imply eventually SCARED or SURPRISED)
```

注意：实际 UPPAAL 语法在开发时按工具规范重写，这里只是性质定义。

---

## 12. 与社交导航算法的接口

自动机完成后，机器人算法可以选择是否使用 social state。

机器人 observation 可扩展为：

\[
o_t=[
Laser,
Goal,
RobotState,
HumanPose,
HumanVelocity,
HumanSocialState
]
\]

第一版先发布 social state，不要求马上修改 PPO 网络。

建议 ROS debug topic：

```text
/formal_social_behavior/states
/formal_social_behavior/transitions
/formal_social_behavior/events
```

如不想立刻增加自定义 msg，可先使用现有消息或 JSON/String 作为调试输出；正式训练前再设计消息类型。

---

## 13. 后续 RL 训练计划

当前仓库已有 Arena-Rosnav PPO / reward / Gymnasium 代码，但现有主要 Gym 环境是 Flatland 环境，不应直接假设 Isaac 六行为 demo 已经是完整 RL Env。

形式化人群模型稳定以后，再新增 Isaac RL wrapper，至少包括：

```text
reset()
step(action)
observation
reward
termination
episode timeout
scene randomization
human reset
robot reset
```

最终训练分三个环境做对比：

```text
A. Fixed HuNav six behaviors
B. Randomized HuNav behaviors
C. Formal Social Automata + HuNav
```

对比：

```text
Success Rate
Collision Rate
Time to Goal
Path Length
Minimum Human Distance
Personal Space Violation
Social Interaction Violation
Episode Return
Generalization
```

不要在当前固定六行为 demo 上直接进行最终大规模 PPO 训练。

---

## 14. 自动机与 Behavior Tree 的定位

本项目当前阶段**不要求升级到 HuNavSim 2.0 Behavior Tree**。

自动机与 BT 的关系：

```text
Automaton
  -> 描述“当前是什么社会/心理状态，以及为什么发生状态变化”

Behavior Tree
  -> 描述“当前条件下具体执行哪些任务/行为”

HuNav/SFM
  -> 描述“具体怎样连续运动”
```

未来可以扩展为：

```text
Formal Psychological Automaton
        ↓
Behavior Tree
        ↓
HuNav / SFM
        ↓
Isaac Sim
```

但第一阶段采用：

```text
Formal Automaton
        ↓
HuNav behavior adapter
        ↓
HuNav / SFM
        ↓
Isaac Sim
```

避免为了 BT 先升级 HuNavSim v2，导致破坏当前稳定环境。

---

## 15. 编码原则

必须遵守：

1. 不破坏当前 baseline。
2. 所有新功能默认可关闭。
3. 阈值全部配置化。
4. Event Extractor、Automaton、HuNav Adapter 必须解耦。
5. 不在 Isaac Character 端加入心理逻辑。
6. 不直接写人物 pose 来模拟行为。
7. 不把状态转移写成散落在多个文件里的大量 `if`。
8. 状态转移必须集中定义并可检查。
9. 每次转移必须可记录和复现。
10. 相同输入 + 相同初始状态必须得到相同输出。
11. 第一版优先确定性自动机，不立即引入概率状态。
12. 不使用 sudo，不污染 HOME 其他目录。
13. 复用当前 Conda / ROS2 / Isaac Sim 环境。
14. 不升级大型依赖，除非确有必要并先报告。

---

## 16. 测试要求

### Unit Tests

至少包括：

```text
test_robot_visible_event
test_ttc_low_event
test_hysteresis
test_normal_to_attention
test_attention_to_scared
test_attention_to_curious
test_scared_recovery
test_no_nondeterministic_transition
test_parallel_shared_event
```

### Integration Tests

至少包括：

```text
1 Robot + 1 Human
- slow safe approach -> Curious
- sudden fast approach -> Scared
- stop/observe case -> Surprised
- robot leaves -> Normal

1 Robot + 2 Humans
- mutual gaze -> SOCIAL
- robot enters interaction space -> break / transition
```

### Regression Tests

关闭自动机功能时必须重新通过：

```text
verify_runtime
verify_six_behaviors
Nav2 smoke test
```

---

## 17. 第一阶段验收标准

第一阶段完成的定义：

- [ ] Baseline 不受影响；
- [ ] 新增 formal social behavior 模块；
- [ ] 事件提取器完成；
- [ ] 1 Robot + 1 Human 自动机完成；
- [ ] 状态转移可配置；
- [ ] 具有 hysteresis / timeout；
- [ ] 自动机状态能动态驱动 HuNav 行为；
- [ ] `NORMAL -> CURIOUS -> NORMAL` 可重复触发；
- [ ] `NORMAL -> SCARED -> NORMAL` 可重复触发；
- [ ] `NORMAL -> SURPRISED -> NORMAL` 可重复触发；
- [ ] transition trace 可记录；
- [ ] 单元测试通过；
- [ ] 六行为 baseline regression 通过；
- [ ] Nav2 regression 通过；
- [ ] 不修改 Isaac Character 社会逻辑；
- [ ] 不升级 HuNavSim v2。

---

## 18. Codex Agent 建议执行顺序

请严格按以下顺序执行，不要一次性重构整个系统：

```text
Step 1
阅读 README.md / HANDOFF.md / DEPLOYMENT.md
确认当前启动和验证流程

Step 2
检查 hunav_six_behaviors_bridge.py
确认 robot / humans / /compute_agents 数据结构

Step 3
实现独立 Event Extractor
只输出日志，不改变行为

Step 4
实现纯 Python Automaton
先完成 unit test

Step 5
接入 bridge，但使用 feature flag
默认关闭

Step 6
验证 runtime HuNav behavior switching
如果不支持，先报告原因，再选择最小适配方案

Step 7
实现 1 Robot + 1 Human demo
保存运行日志和状态转移 trace

Step 8
回归测试原六行为和 Nav2

Step 9
实现 2 Human shared-event prototype

Step 10
更新 HANDOFF.md
记录新模块、启动命令、验证结果和已知问题
```

---

## 19. 不要做的事情

本任务第一阶段明确不做：

```text
- 不把 HuNav v1 全面升级成 HuNavSim v2
- 不直接引入 Behavior Tree 取代自动机
- 不开始最终 PPO 大规模训练
- 不修改 Jackal D6 控制器
- 不修改 Nav2 规划器
- 不修改 Isaac Sim 人物根节点控制逻辑
- 不做几十人的复杂群体
- 不立即加入概率自动机
- 不立即加入深度学习心理模型
```

目标是先得到一个：

\[
\boxed{
Robot + Human
\rightarrow
Formal Event
\rightarrow
Automaton State
\rightarrow
HuNav Behavior
\rightarrow
Continuous Motion
}
\]

的稳定、可重复、可验证闭环。

---

## 20. 最终目标架构

```text
                     Isaac Sim
                         │
          ┌──────────────┴──────────────┐
          │                             │
       Robot state                  Human states
          │                             │
          └──────────────┬──────────────┘
                         ↓
                  Event Extractor
                         ↓
            Formal Social Automata
            H1 || H2 || ... || HN
                         ↓
                Social/Psych State
                         ↓
               HuNav Behavior Adapter
                         ↓
                     HuNav SFM
                         ↓
                Human next states
                         ↓
                     Isaac Sim

                     Robot side
                         │
            perception / social state
                         ↓
              Social Navigation
              DRL / MPC / DRL+MPC
                         ↓
                      /cmd_vel
                         ↓
                       Jackal
                         │
                         └────────────── feedback ──────────────┐
                                                               │
                    Human automata receive robot behavior <────┘
```

最终研究问题不是“机器人避开随机行人”，而是：

> **在机器人参与的动态人群交互中，使用形式化自动机描述人的社会/心理状态演化，并研究机器人如何在这种可解释、可验证的人机交互环境中完成社会导航。**

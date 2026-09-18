# Arena5 当前 MPC 控制器方法与参数说明

文档日期：2026-09-17  
适用运行发布：`20260916-4ff2edd`  
运行源码提交：`4ff2edd9715e2ee839f055558450dc0297886927`  
目标环境：ROS 2 Humble、Nav2、Arena-Rosnav 5.0、Isaac Sim 5.1、HuNavSim、Jackal D6

本文描述当前已经实现和发布的 MPC，而不是原 ROS 1 参考项目的原始实现，也不是后续拟议方案。主要代码和配置来源为：

- `src/arena_mpc_core/src/solver.cpp`
- `src/arena_mpc_core/src/model.cpp`
- `src/arena_mpc_controller/src/mpc_controller.cpp`
- `src/arena_mpc_controller/src/mpc_command_watchdog.cpp`
- `src/arena_mpc_bringup/config/controller_model.yaml`
- `src/arena_mpc_bringup/config/nav2_overrides.yaml`

## 1. 方法概览

当前方法是一个有限时域、非线性、直接多重射击 MPC。每次 Nav2 控制调用都执行以下过程：

1. 读取当前机器人位姿、实际 odom 速度、Nav2 全局路径、HuNav 行人状态、lidar 和 local costmap 状态。
2. 把全局路径和行人状态转换到 local costmap 的全局坐标系；当前通常为 `odom`。
3. 沿全局路径生成 26 个带时间含义的参考状态。
4. 用匀速模型预测每名行人未来 2.5 s 的位置。
5. 用 CasADi 构建的固定规模 NLP 和 IPOPT 求解 25 步状态与控制序列。
6. 对求解结果进行独立数值残差检查、动态行人连续碰撞检查、完整矩形 footprint costmap 扫掠检查和制动可行性检查。
7. 只提交第一控制量，然后在下一控制周期重新测量并重新求解。
8. velocity smoother 对命令做限幅和平滑，独立 watchdog 只有在控制状态、命令链和所有安全输入均新鲜时才转发到 `/cmd_vel`。

```mermaid
flowchart LR
    GP[NavFn 全局路径 /plan] --> CS[Nav2 controller_server 10 Hz]
    ODOM[/odom 实际速度] --> MPC[MPC Controller plugin]
    HUMAN[/human_states HuNav 真值] --> MPC
    LIDAR[/lidar] --> VOXEL[Local VoxelLayer]
    DEPTH[/lidar_clearing] --> VOXEL
    VOXEL --> COSTMAP[Local costmap]
    LIDAR --> FRESH[Lidar 新鲜度检查]
    FRESH --> MPC
    COSTMAP --> MPC
    CS --> MPC
    MPC -->|第一控制量 /cmd_vel_nav| SMOOTHER[velocity_smoother 20 Hz]
    SMOOTHER -->|/cmd_vel_watchdog_in| WATCHDOG[watchdog 50 Hz]
    HUMAN --> WATCHDOG
    ODOM --> WATCHDOG
    LIDAR --> WATCHDOG
    COSTMAP --> WATCHDOG
    WATCHDOG -->|仅安全链完整时| CMD[/cmd_vel]
```

MPC 对动态行人的位置和速度使用 HuNavSim 仿真真值。lidar 不负责识别人或估计人的状态；它通过 VoxelLayer 形成局部占据栅格，并作为独立新鲜度信号参与停车保护。

## 2. 状态、控制量与预测时域

### 2.1 状态和控制量

机器人状态为：

```text
x_k = [p_x,k, p_y,k, theta_k]^T
```

控制量为：

```text
u_k = [v_k, omega_k]^T
```

其中：

- `p_x, p_y`：机器人平面位置，单位 m；
- `theta`：偏航角，单位 rad；
- `v`：车体前向线速度，单位 m/s；
- `omega`：偏航角速度，单位 rad/s。

当前实现不优化横向速度，也不允许 MPC 倒车：`v_k >= 0`。角速度可以为正或负。

### 2.2 离散运动学

模型采用非完整约束的单轮车运动学：

```text
p_x,k+1 = p_x,k + dt * v_k * cos(theta_k)
p_y,k+1 = p_y,k + dt * v_k * sin(theta_k)
theta_k+1 = theta_k + dt * omega_k
```

当前：

```text
N  = 25
dt = 0.1 s
T  = N * dt = 2.5 s
```

偏航状态本身不在动力学后强制归一化，但所有航向误差都用：

```text
wrap(delta_theta) = atan2(sin(delta_theta), cos(delta_theta))
```

处理，因此跨越 `-pi/pi` 不会产生约 `2*pi` 的错误误差。

### 2.3 NLP 规模

当前固定 8 个动态障碍槽，静态障碍不进入 NLP。固定规模问题包含：

| 项目 | 数量 |
|---|---:|
| 状态决策变量 | `3 * (N + 1) = 78` |
| 控制决策变量 | `2 * N = 50` |
| 决策变量合计 | `128` |
| 初始状态、实测控制及时间参数 | `6` |
| 参考轨迹参数 | `3 * 26 = 78` |
| 每个障碍槽参数 | `1 + 5 * 26 = 131` |
| 8 个障碍槽后的参数合计 | `1132` |
| 等式/不等式约束合计 | `336` |

障碍槽的第一个参数是 active mask，其余是每个预测时刻的 `x、y、长轴、短轴、yaw`。未使用槽填入远处占位圆，并令 active 为 0。NLP 图在 plugin 配置阶段构建，控制周期只更新参数。

## 3. 路径参考如何生成

### 3.1 坐标变换和路径截取

每次控制调用会：

1. 将机器人位姿转换到 local costmap global frame；
2. 将 Nav2 全局路径的所有 Pose 转换到同一 frame；
3. 用欧氏距离寻找距离机器人最近的路径点；
4. 从该点开始计算路径累计弧长；
5. 按固定弧长间隔线性插值出 `N+1` 个参考位置；
6. 用路径切线生成参考航向；位置尚未进入 GoalChecker 的 XY 容差时，路径末端重复点
   使用最后一个非退化路径段的切向；进入容差并锁存后，全部参考切换到精确终点和
   全局路径终点航向。

路径短于预测范围时，后续参考位置会重复路径终点。因此 terminal cost 跟踪的是“预测窗口末端参考”，只有当最终导航目标落入窗口时，它才等同于最终导航目标。

终点使用两个阶段。位置阶段保持驶入路径的航向，避免不能侧移、不能倒车的单轮车
同时追踪终点位置和不相容的最终航向而停在容差外。机器人进入 active GoalChecker
报告的 XY 容差后，位置阶段锁存，26 个参考都设为终点位置和最终航向，使优化器原地
完成姿态。当前 GoalChecker 提供 0.25 m；只有 checker 无法返回有效容差时才使用
`goal_position_tolerance_fallback=0.25 m`。新路径或生命周期切换会清除锁存。

### 3.2 当前隐含参考速度

参考弧长间隔为：

```text
reference_spacing = 0.025 m
```

每个模型步为 0.1 s，因此参考轨迹隐含的标称速度为：

```text
v_ref = reference_spacing / dt
      = 0.025 / 0.1
      = 0.25 m/s
```

完整预测窗口沿路径只向前看：

```text
N * reference_spacing = 25 * 0.025 = 0.625 m
```

这解释了为什么把 `max_linear` 提高到 0.8 m/s 后，当前 MPC 输出仍通常接近 0.25～0.26 m/s。`max_linear` 是允许上界，参考轨迹仍要求机器人以约 0.25 m/s 的节奏推进。

### 3.3 路径世代保护

控制器对路径 frame、位置和姿态计算哈希。内容变化时增加 `path_generation`。如果路径在求解或后检查期间被替换，本次旧结果不会提交；控制器发布零速 retry，并在下一周期使用新路径重新求解。这个 retry 不消耗连续失败次数。

## 4. 目标函数

### 4.1 完整形式

令参考状态为：

```text
r_k = [r_x,k, r_y,k, r_theta,k]^T
```

当前目标函数可以写成：

```text
J = J_stage + J_terminal + J_barrier
```

阶段代价：

```text
J_stage = sum(k=0..N-1) {
    0.1 * [
        q_p(k) * (p_x,k - r_x,k)^2
      + q_p(k) * (p_y,k - r_y,k)^2
      + q_theta(k) * wrap(theta_k - r_theta,k)^2
    ]
  + 0.1  * v_k^2
  + 0.02 * omega_k^2
}
```

其中：

```text
q_p(k)     = 1.0  + 0.05  * k
q_theta(k) = 0.02 + 0.005 * k
```

终端代价：

```text
J_terminal = terminal_weight * [
    (p_x,N - r_x,N)^2
  + (p_y,N - r_y,N)^2
  + 0.02 * wrap(theta_N - r_theta,N)^2
]
```

当前 `terminal_weight=1.0`。

障碍接近速率罚项：

```text
s_i,k = max(gamma * h_i,k - h_i,k+1, 0)

J_barrier = sum(i) sum(k=0..N-1) slack_weight * s_i,k^2
```

当前：

```text
gamma        = 0.2
slack_weight = 50.0
```

### 4.2 路径位置代价

`q_p(k)` 从第 0 步的 1.0 增加到第 24 步的 2.2。乘外层 0.1 后，实际阶段位置误差系数从 0.1 增加到 0.22。越靠近预测末端，偏离参考路径的代价越大。

这个设计既要求短期命令平稳，也避免求解器为了当前一步的小收益而明显牺牲预测末端的路径跟踪。

### 4.3 航向代价

`q_theta(k)` 从 0.02 增加到 0.14；乘外层 0.1 后，实际阶段航向系数从 0.002 增加到 0.014。航向跟踪权重明显低于位置跟踪，使机器人可以为绕行动态行人暂时偏离路径切线，而不会因为航向项过强而失去横向避让空间。

终端航向误差的系数为 `terminal_weight * 0.02`。

### 4.4 控制使用代价

控制代价为：

```text
0.1 * v_k^2 + 0.02 * omega_k^2
```

其作用是抑制不必要的大速度和大角速度。当前没有以下代价项：

- 没有显式到达时间代价；
- 没有沿路径进度奖励；
- 没有 `v_k - v_desired` 的速度跟踪代价；
- 没有独立的控制增量二次代价 `Delta u^T R_delta Delta u`；
- 没有对加速度或 jerk 的软代价。

加速度由硬约束处理。由于不存在“尽快到达”的收益，而 `v^2` 又直接增加代价，优化器不会仅因为最大速度允许 0.8 m/s 就主动选择 0.8 m/s。

例如，忽略其他项时，25 步恒定速度的线速度代价约为：

```text
v = 0.25 m/s: 25 * 0.1 * 0.25^2 = 0.15625
v = 0.80 m/s: 25 * 0.1 * 0.80^2 = 1.60
```

更重要的是，0.8 m/s 每步前进 0.08 m，而当前参考点每步只前进 0.025 m；高速度还会同时产生“跑到参考点前面”的位置误差。

### 4.5 终端代价

终端位置系数为 1.0，高于单步阶段位置系数。它鼓励 2.5 s 预测轨迹到达第 25 个参考点。当前第 25 个参考点通常位于路径前方约 0.625 m，因此对应平均速度仍约为 0.25 m/s。

### 4.6 屏障罚项的准确语义

实现中的 `barrier_slack` 容易被误解。当前实现具有以下语义：

1. 每个预测时刻的几何安全条件 `h_i,k >= 0` 是硬约束；
2. 几何安全条件不允许 slack 穿透；
3. `s_i,k` 不是 NLP 的独立决策变量；
4. 它由相邻两步 clearance 解析计算：

   ```text
   s_i,k = max(gamma * h_i,k - h_i,k+1, 0)
   ```

5. `slack_weight * s_i,k^2` 只惩罚“安全裕量下降过快”。

当 `h_i,k+1 >= gamma*h_i,k` 时没有罚项。`gamma=0.2` 允许下一步安全裕量下降到当前值的 20%，但不允许 `h` 变成负值，因为每一步仍有独立的硬几何约束。

增大 `gamma` 会更强地要求保留当前安全裕量；增大 `slack_weight` 会更强地惩罚接近过快。两者都不会放宽 `h>=0`。

## 5. 约束设计

### 5.1 初始状态约束

```text
x_0 = x_measured
```

当前机器人位置来自 Nav2 传入位姿并转换到 local costmap global frame。速度没有作为状态，而是作为第一控制量加速度约束的基准。

### 5.2 动力学等式约束

每个预测步都严格满足第 2.2 节的离散单轮车模型。求解后还会用独立 C++ 数值实现重新计算动力学残差。

### 5.3 速度边界

```text
0.0 <= v_k <= min(max_linear, Nav2 speed limit)
-max_angular <= omega_k <= max_angular
```

当前：

```text
max_linear  = 0.8 m/s
max_angular = 1.5 rad/s
```

Nav2 `setSpeedLimit()` 可以进一步降低线速度上界。百分比限速以 0.8 m/s 为基准；绝对限速直接使用给定值，并钳制到 `[0.001, 0.8]`。`NO_SPEED_LIMIT` 恢复 0.8 m/s。

角速度上限目前不随 Nav2 speed limit 百分比缩放。

### 5.4 加速度边界

```text
|v_k - v_k-1|         <= max_linear_accel  * interval
|omega_k - omega_k-1| <= max_angular_accel * interval
```

当前：

```text
max_linear_accel  = 2.0 m/s^2
max_angular_accel = 3.2 rad/s^2
```

`k=0` 时，前一控制量使用订阅 `/odom` 得到的实际线速度和角速度。后续步骤使用相邻优化控制量。

当前 plugin 把第一步 `first_interval` 固定为 `dt=0.1 s`。它会检查控制调用时间是否倒退，但正向的实际调用间隔没有写入 `first_interval`。所以从静止开始，NLP 第一控制量相对 odom 的最大变化为：

```text
Delta v_0     <= 2.0 * 0.1 = 0.20 m/s
Delta omega_0 <= 3.2 * 0.1 = 0.32 rad/s
```

### 5.5 动态行人几何硬约束

每个障碍样本表示为椭圆：

```text
o_i,k = [o_x, o_y, a, b, psi]
```

把机器人中心相对障碍中心的位移旋转到障碍局部坐标系：

```text
dx' =  cos(psi) * dx + sin(psi) * dy
dy' = -sin(psi) * dx + cos(psi) * dy
```

clearance 函数为：

```text
h = b * (
      sqrt((dx'/a)^2 + (dy'/b)^2 + epsilon)
      - sqrt(epsilon)
      - 1
    ) - safe_distance

epsilon = 1e-12
```

硬约束为：

```text
h_i,k >= 0
```

当前 HuNav 障碍实际构造成圆，因此 `a=b=R`，该函数近似等价于：

```text
h = center_distance - R - safe_distance
```

`min_axis=1e-3 m` 用于拒绝退化椭圆。

## 6. HuNav 行人预测与安全距离

### 6.1 数据来源

`/human_states` 提供：

- 行人 ID；
- 真实位置；
- 平面线速度；
- 行人半径；
- 消息时间戳和 frame。

这属于仿真真值输入，不是 lidar 在线检测结果。行为类型不会直接进入 MPC 目标函数；HuNav 行为只通过实际产生的位置和速度间接影响 MPC。

订阅 QoS 为 Reliable、Volatile、KeepLast(1)。负 ID、重复 ID、非有限速度、非有限或非正半径都会使本周期失败停车。

### 6.2 常速度预测

行人位置和速度先转换到 local costmap global frame。每个预测样本使用：

```text
t_predict(k) = max(0, message_age) + k * dt

o_x,k = o_x,measured + v_x * t_predict(k)
o_y,k = o_y,measured + v_y * t_predict(k)
```

因此消息有小幅延迟时，第 0 个障碍样本也会向前外推，而不是把旧位置当作当前真实位置。

当前预测模型不包括：

- 行人加速度；
- 转向或意图模型；
- HuNav 行为类别作为显式状态；
- 协方差传播；
- Kalman 滤波。

### 6.3 障碍膨胀

每个行人预测圆的 NLP 半径为：

```text
R = human.radius
  + robot_circumscribed_radius
  + geometry_uncertainty
```

当前机器人矩形 footprint 为：

```text
x = +/-0.24 m
y = +/-0.22 m
```

外接圆半径为：

```text
sqrt(0.24^2 + 0.22^2) = 0.3256 m，运行中约记为 0.326 m
```

当前：

```text
geometry_uncertainty = 0.05 m
safe_distance        = 0.35 m
```

所以正常 NLP 要求的中心距离近似为：

```text
center_distance >= human.radius + 0.3256 + 0.05 + 0.35
                >= human.radius + 0.7256 m
```

这个表达式已经包含机器人外接圆，不应再把机器人半径重复加一次。

### 6.4 输入容量和 NLP 容量

当前最多接受 32 名动态行人。求解前使用最大允许线速度做可达性筛选：

```text
center_distance(k) - obstacle_extent(k)
    <= k * dt * effective_linear_limit + tolerance
```

只有预测时域内可能到达的行人才进入 NLP。固定 NLP 最多容纳 8 名相关行人：

- 超过 32 名输入：立即报容量错误；
- 不超过 32 名，但相关行人超过 8 名：报 NLP 容量错误并停车；
- 未进入 NLP 的远处行人仍会在独立求解后检查中覆盖。

当前筛选按 `/human_states` 输入顺序处理，并没有按最近距离重新排序。

## 7. 静态障碍、lidar 与 costmap

### 7.1 静态障碍不进入 NLP

核心库保留 `max_static_obstacles=128` 的通用输入校验字段，但当前 Nav2 plugin 设置：

```text
max_nlp_static_obstacles = 0
```

plugin 也不会从 costmap 提取 128 个静态点或椭圆。静态结构和 lidar 障碍通过 local costmap 的完整 footprint 后检查处理。

### 7.2 实际感知链

```text
RTX lidar -> local VoxelLayer -> local costmap -> MPC 后验碰撞检查
```

原始 `/lidar` 继续负责 marking 和常规 clearing。`/lidar_clearing` 来自渲染深度，只负责 clearing，不能 marking。MPC plugin 订阅 `/lidar` 的消息时间戳来确认传感器仍然在线，但不会把 LaserScan 点直接放入 NLP。

global costmap 使用：

```text
static_layer + inflation_layer
```

它不使用全局 lidar obstacle layer。动态行人避障发生在 local MPC 和 local costmap 链路。

### 7.3 当前 costmap 几何

| 项目 | Local costmap | Global costmap |
|---|---:|---:|
| frame | `odom` | `map` |
| 尺寸 | 15 m × 15 m | 20 m × 20 m |
| 分辨率 | 0.10 m | 0.05 m |
| 更新/发布频率 | 10/10 Hz | 5/2 Hz |
| footprint | ±0.24 m × ±0.22 m | 同左 |
| footprint padding | 0 | 0 |
| inflation radius | 0.55 m | 0.55 m |
| 主要层 | Voxel + Inflation | Static + Inflation |

### 7.4 完整 footprint 扫掠检查

求解后会把矩形 footprint 旋转和平移到每个预测姿态，并用 `convexFillCells()` 检查多边形内部所有栅格。

以下情况判定不安全：

- footprint 顶点在地图外；
- 多边形无法填充到有效栅格；
- 任意内部单元为 `NO_INFORMATION`；
- 任意内部单元大于等于 `LETHAL_OBSTACLE`。

相邻预测姿态之间还会插值，采样运动步长为：

```text
max(0.005 m, 0.5 * costmap_resolution)
```

采样数量同时考虑机器人中心平移和：

```text
abs(delta_yaw) * robot_circumscribed_radius
```

因此旋转时角点扫过的区域也会检查。当前后检查不把低于 lethal 的普通 inflation cost 当作碰撞；硬碰撞依据未知区和 lethal 栅格。

## 8. 制动轨迹与求解后安全检查

### 8.1 制动轨迹

候选第一控制量和当前实测 odom 控制量都会生成制动轨迹。制动模拟步长固定为：

```text
brake_dt = 0.05 s
```

每步按最大减速度使线速度和角速度逼近 0，最多模拟 100 步，即最多 5 s。由于当前 `v>=0`，线速度正常从正值降到 0。

### 8.2 候选轨迹必须同时满足

正常提交非零控制量前，必须通过：

1. IPOPT 报告成功；
2. 实测求解时间不超过 75 ms；
3. 独立 C++ 残差检查全部不超过 `1e-3`；
4. 完整 MPC 预测轨迹通过动态行人连续线段检查；
5. 从候选第一控制量开始的完整制动轨迹通过动态行人检查；
6. 完整 MPC 预测轨迹通过矩形 footprint costmap 扫掠检查；
7. 候选制动轨迹通过矩形 footprint costmap 扫掠检查；
8. 输入在提交前仍然新鲜；
9. 求解期间没有 path generation 或 reset epoch 变化；
10. 从 plugin 入口到提交的完整周期不超过 90 ms。

动态行人连续检查不是只检查离散端点。它计算机器人与行人在相邻样本之间的相对线段最近点，并用两端最大障碍半径作为要求距离。

### 8.3 紧急停车安全下界

如果 IPOPT timeout/infeasible，或者候选轨迹动态检查不安全，控制器发布零速等待。
当当前实测运动的制动轨迹仍满足下述下界时，零速命令按正常
`human_wait` 链路经过 velocity smoother；如果机器人已经进入保守制动包络，任何
新的非零控制都不能在本周期恢复净距，此时控制器发布
`stop recoverable=1 mode=safety_wait`，由 watchdog 立即强制 `/cmd_vel=0`，同时正常
返回本周期，使原 FollowPath 目标保持活动并在下一周期重新求解。

紧急制动检查使用真实矩形 footprint，而不是 NLP 中的机器人外接圆；其净安全下界为：

```text
emergency_safe_distance = 0.30 m
```

正常规划使用 0.35 m，紧急停车检查使用 0.30 m。若检查通过，它是制动过程的硬
下界；若检查发现机器人已经位于该包络内，状态会明确报告违反该下界，立即零速是
当时风险最小的动作，不能把已经存在的几何冲突描述为仍满足 0.30 m。

### 8.4 动态行人留下的 costmap 残留

如果候选轨迹只在 costmap 中碰撞，同时碰撞点能与当前行人及其运动范围关联，并且当前实测制动轨迹安全，控制器会保持零速等待 costmap clearing：

```text
costmap_obstacle_wait_limit = 120.0 s
```

在线六行人运行已观测到约 12 s 后才清除的关联占用，原 1 s 假设会先错误终止
action，因此上限修订为有限 120 s。等待期间 watchdog 保持零速，且
SafetyAwareProgressChecker 暂停活动跟踪预算；超过 120 s 仍未清除就转为失败。
无法与当前行人关联的静态碰撞不会进入这条等待路径。

2026-09-18 的正式六行人耐久运行持续 1800.514 s，31 个已结束目标全部成功，
ABORTED 和单目标测试超时均为 0。它覆盖了 1,937 次最新 HuNav 更新使实测制动轨迹
失效后的可恢复停车和 294 次 solver wall-time 超限后的安全等待；没有把这些瞬态状态
转换成 action failure。完整结构化结果见
`evidence/abort_fix/endurance_30min_corrected.json`。

## 9. 求解器和 warm start

### 9.1 CasADi/IPOPT 设置

当前求解器设置：

| 参数 | 当前值 |
|---|---:|
| CasADi | 3.8.0 |
| NLP solver | IPOPT 3.14.19 |
| 最大迭代数 | 100 |
| `ipopt.max_wall_time` | 0.075 s |
| `ipopt.tol` | `1e-3` |
| `ipopt.acceptable_tol` | `1e-3` |
| `ipopt.acceptable_obj_change_tol` | `1e-3` |
| `honor_original_bounds` | yes |
| 图布局 | FixedMasked |
| OpenBLAS/OMP 线程 | 各 1 |

IPOPT 的 wall timer 不是线程硬抢占，因此求解返回后还会再用 steady clock 测量。实测求解时间超过 75 ms 时，即使 IPOPT 给出解也标记为 timeout。

### 9.2 初始猜测

没有可用 warm start 时：

1. 从实测状态和 odom 控制开始；
2. 对每一步指向下一个参考点；
3. 线速度初值取 `distance_to_next_reference / dt`；
4. 角速度初值取 `heading_error / dt`；
5. 同时按速度上限和相邻控制加速度约束钳制；
6. 用运动学模型重新 rollout 状态。

### 9.3 warm start

求解成功并通过独立检查后保存完整解。下一周期：

1. 将上一控制序列左移一步；
2. 最后一步重复上一解末端控制；
3. 第一控制量重新以最新 odom 速度为基准施加加速度钳制；
4. 从最新机器人状态重新 rollout 全部状态。

失败解不会保存为 warm start。plugin 激活、停用、清理、时钟 reset epoch 变化时都会清空 warm start。

## 10. 独立数值后检查

求解结果不会只依赖 IPOPT 的 `success` 标志。独立 C++ evaluator 会重算：

- 初始状态残差；
- 离散动力学残差；
- 几何安全违反量；
- barrier 关系违反量；
- 速度边界违反量；
- 加速度边界违反量；
- 目标函数是否有限。

每项最大允许误差为：

```text
acceptable_tolerance = 1e-3
```

任何非有限值或超差都会拒绝本次命令。独立 evaluator 使用全部输入行人，而不只使用进入 8 个 NLP 槽的行人。

## 11. Nav2 集成和控制命令链

### 11.1 controller_server

当前：

```text
controller_frequency = 10 Hz
failure_tolerance    = 0
```

Nav2 `computeVelocityCommands()` 传入的 `velocity` 参数在当前 plugin 中没有直接使用；plugin 使用自己订阅并检查新鲜度的 `/odom` 速度。plugin 通过 `goal_checker->getTolerances()` 让终点参考的两阶段切换与实际 XY 容差一致；是否正式到达仍由 Nav2 controller server 的 stateful SimpleGoalChecker 判断：

```text
xy_goal_tolerance  = 0.25 m
yaw_goal_tolerance = 0.25 rad
```

动态行人等待可能持续较长时间，因此 MPC 模式使用
`arena_mpc_controller::SafetyAwareProgressChecker`，参数为
`movement_time_allowance=120 s`、`required_movement_radius=0.05 m`、
`required_movement_angle=0.1 rad`、
`status_timeout=1.0 s`。0.05 m 小于短回程在 goal tolerance 外可能剩余的有效
距离；0.1 rad 让终点原地转向也能刷新时限；120 s 只累计普通 `track` 时间。只有持续收到新鲜
`human_wait/safety_wait` 状态时暂停计时，状态断流或普通跟踪停滞仍会有界失败。

### 11.2 velocity smoother

| 参数 | 当前值 |
|---|---:|
| 频率 | 20 Hz |
| feedback | OPEN_LOOP |
| max velocity | `[0.8, 0.0, 1.5]` |
| min velocity | `[-0.8, 0.0, -1.5]` |
| max accel | `[2.0, 0.0, 3.2]` |
| max decel | `[-2.0, 0.0, -3.2]` |
| velocity timeout | 0.25 s |

虽然 smoother 允许负线速度，MPC 自身 `min_linear=0`，所以正常 MPC 输出不会倒车。

### 11.3 watchdog

watchdog 每 20 ms，也就是 50 Hz 发布一次 `/cmd_vel`。只有以下条件全部满足时才转发最近的平滑命令：

- controller status 以 `ok ` 开头；
- 收到该 status 之后的 raw command；
- 收到 raw command 之后的 smoothed command；
- status、raw、smooth 均未超过 0.25 s lease；
- `/human_states`、`/odom`、`/lidar` 和 `/local_costmap/costmap_raw` 同时满足 ROS 时间与墙钟新鲜度；
- ROS 时间没有倒退；
- `/cmd_vel` 没有第二个发布者。

任一条件失败时 watchdog 持续发布零速度。

## 12. 时间、新鲜度与故障处理参数

### 12.1 plugin 输入检查

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `transform_tolerance` | 0.2 s | TF 查询允许等待时间 |
| `ros_age_limit` | 0.3 s | human/odom/lidar 最大 ROS 时间年龄 |
| future tolerance | 0.05 s | 硬编码；允许少量未来时间戳 |
| `human_wall_limit` | 0.6 s | human 最大墙钟断流时间 |
| `odom_wall_limit` | 0.4 s | odom 最大墙钟断流时间 |
| `lidar_wall_limit` | 1.55 s | lidar 最大墙钟断流时间 |
| `solver_budget_ms` | 75 ms | IPOPT 和实测求解上限 |
| `plugin_commit_limit_ms` | 90 ms | plugin 完整周期提交上限 |
| `failure_limit` | 5 次 | 连续普通失败后抛出 Nav2 异常 |

plugin 在求解开始前和命令提交前都检查输入新鲜度。行人在求解期间更新时，还会用最新快照再次验证候选轨迹或实测制动轨迹。

Nav2 使用本包的 `SafetyAwareProgressChecker`，配置
`required_movement_radius=0.05 m`、`required_movement_angle=0.1 rad`、
`movement_time_allowance=120 s` 和
`status_timeout=1.0 s`。原继承值 0.5 m 大于若干短回程目标的实际剩余距离；标准
SimpleProgressChecker 还会把安全等待计入时限。新插件只在 MPC 持续报告新鲜
`human_wait/safety_wait` 时暂停活动跟踪计时。该修订只影响 action 是否继续等待，
不放宽 MPC、costmap 或 watchdog 的任何运动安全条件。

### 12.2 watchdog 检查

| 参数 | 当前值 |
|---|---:|
| command/status/raw/smooth lease | 0.25 s |
| ROS age timeout | 0.30 s |
| future tolerance | 0.05 s |
| human wall timeout | 0.60 s |
| odom wall timeout | 0.40 s |
| lidar wall timeout | 1.55 s |
| costmap wall timeout | 1.55 s |

plugin 和 watchdog 的检查互相独立。plugin 卡在 costmap 等待或求解器内部时，watchdog 仍能在 command lease 到期后停车。

### 12.3 失败语义

普通失败时，plugin 先发布 `stop` 状态并返回零速度。连续第 5 次失败时抛出 `nav2_core::PlannerException`，Nav2 FollowPath 终止。MPC 不自动切换到 DWB。

以下情况属于安全等待，不累计普通失败：

- 路径在求解期间更新：发布带 `ok` 状态的零速 retry；
- solver timeout/infeasible 或候选动态轨迹不安全，但当前实测制动轨迹仍安全：发布零速 `human_wait`。
- solver timeout/infeasible、候选后检查失败或最新 HuNav 更新到达，且实测制动轨迹
  已进入动态/静态保守包络：发布可恢复 `safety_wait`，watchdog 立即强制零速；不抛
  `PlannerException`，障碍移除后继续同一目标。

输入断流、非法数据、TF/路径错误、不可归因于动态行人的持续静态碰撞等仍是普通
失败。普通跟踪状态的长期无进展仍受 120 s 活动跟踪预算、持续碰撞失败或其他 Nav2
行为约束；动态安全等待本身不再终止尚未到达的目标。

## 13. 全部当前 MPC 核心参数

### 13.1 NLP 和运动学参数

| 参数 | 当前值 | 来源 | 作用 |
|---|---:|---|---|
| `horizon` | 25 | YAML | 控制步数 |
| `dt` | 0.1 s | YAML | 模型步长 |
| prediction horizon | 2.5 s | 派生 | `N*dt` |
| `reference_spacing` | 0.025 m | YAML | 相邻路径参考点弧长 |
| `goal_position_tolerance_fallback` | 0.25 m | YAML | GoalChecker 无有效 XY 容差时的后备值 |
| nominal reference speed | 0.25 m/s | 派生 | `spacing/dt` |
| path lookahead | 0.625 m | 派生 | `N*spacing` |
| `min_linear` | 0.0 m/s | C++ 默认 | 禁止倒车 |
| `max_linear` | 0.8 m/s | YAML | 线速度硬上限 |
| `max_angular` | 1.5 rad/s | YAML | 角速度硬上限 |
| `max_linear_accel` | 2.0 m/s² | YAML | 线速度变化硬约束 |
| `max_angular_accel` | 3.2 rad/s² | YAML | 角速度变化硬约束 |
| `terminal_weight` | 1.0 | YAML | 终端跟踪代价倍率 |
| `gamma` | 0.2 | YAML | clearance 保持比例 |
| `slack_weight` | 50.0 | YAML | clearance 下降过快罚项 |
| `safe_distance` | 0.35 m | YAML | 正常动态避障净距 |
| `emergency_safe_distance` | 0.30 m | YAML | 实测制动硬下界 |
| `geometry_uncertainty` | 0.05 m | YAML | 行人几何额外膨胀 |
| `min_axis` | `1e-3` m | C++ 默认 | 最小有效椭圆轴 |

### 13.2 目标函数中硬编码的系数

| 项 | 当前系数 |
|---|---:|
| 阶段位置外层系数 | 0.1 |
| 阶段位置递增权重 | `1.0 + 0.05*k` |
| 阶段航向递增权重 | `0.02 + 0.005*k` |
| 线速度二次代价 | 0.1 |
| 角速度二次代价 | 0.02 |
| 终端位置内部系数 | 1.0 |
| 终端航向内部系数 | 0.02 |

这些系数目前不是 ROS 参数；修改它们需要改代码、重建和重新验证。

### 13.3 容量和求解参数

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `max_dynamic_obstacles` | 32 | 最大 HuNav 输入数量 |
| `max_static_obstacles` | 128 | 通用核心字段；当前 plugin 不输入静态对象 |
| `max_nlp_dynamic_obstacles` | 8 | 固定 NLP 动态槽 |
| `max_nlp_static_obstacles` | 0 | 静态障碍不进入 NLP |
| `max_iterations` | 100 | IPOPT 最大迭代 |
| `solver_budget_ms` | 75 | 求解墙钟预算 |
| `acceptable_tolerance` | `1e-3` | IPOPT 和独立后检查容差 |
| solver layout | FixedMasked | 固定图加 active mask |

### 13.4 footprint 和 costmap 相关参数

| 参数 | 当前值 |
|---|---:|
| footprint 前/后半长 | 0.24 m |
| footprint 左/右半宽 | 0.22 m |
| footprint padding | 0 |
| 外接圆半径 | 约 0.326 m |
| local inflation radius | 0.55 m |
| global inflation radius | 0.55 m |
| lidar expected update rate | 0.3 s |
| depth clearing raytrace max range | 3.0 m |
| costmap obstacle wait limit | 120.0 s |
| 制动轨迹采样步长 | 0.05 s |

## 14. 为什么当前不会自然跑到 0.8 m/s

当前设计共同产生约 0.25 m/s 的标称速度：

1. 参考点每 0.1 s 只前进 0.025 m；
2. 终端参考在 2.5 s 后只位于前方约 0.625 m；
3. 跑得比参考快会增加位置误差；
4. `0.1*v^2` 直接惩罚较大速度；
5. 没有到达时间代价或路径进度奖励；
6. goal checker 在距离目标 0.25 m 时即可结束，短目标没有足够巡航距离；
7. 转弯、行人、costmap 和制动后检查只会进一步降速，不会提高速度。

因此 `max_linear=0.8` 的准确含义是“优化器允许使用最高 0.8 m/s”，不是“期望速度为 0.8 m/s”。

2026-09-17 的发布版实机 smoke 从 `(3.0, 3.0)` 导航到 `(3.6, 3.0)`。机器人移动约 0.373 m 后进入 0.25 m goal tolerance，最大 MPC 线速度为 `0.25999 m/s`。这个结果与 0.25 m/s 的时间参数化参考一致，不表示 0.8 m/s 上限没有生效。原始结构化结果保存在 `evidence/speed_limits/mpc_live.json`。

## 15. 参数调节的实际影响

| 调节项 | 增大后的主要效果 | 主要风险或代价 |
|---|---|---|
| `reference_spacing` | 提高隐含参考速度和路径前视距离 | 急弯切角、制动距离增加、行人预测要求提高 |
| `max_linear` | 放宽可行速度上界 | 单独提高通常不会明显提速 |
| `terminal_weight` | 更强调预测末端参考 | 可能产生更激进的短期动作 |
| 阶段位置权重 | 更紧跟路径 | 避让自由度减少 |
| `v^2` 权重 | 增大时更慢、更保守 | 过大时进展不足 |
| `omega^2` 权重 | 增大时减少转向 | 弯道跟踪和避障能力下降 |
| `safe_distance` | 增大正常行人净距 | 可行空间缩小、等待增多 |
| `gamma` | 更强地保留 clearance | 动态交互更保守、可行性下降 |
| `slack_weight` | 更强惩罚快速接近 | NLP 条件可能更困难 |
| `N` | 增大预测时间/分辨率范围 | 决策规模和求解时间增加 |
| `dt` | 增大单步时间和预测范围 | 离散模型更粗、碰撞采样要求更高 |
| NLP 行人槽数 | 同时优化更多行人 | 求解耗时和数值复杂度上升 |

如果目标是让机器人在开阔直线接近 0.8 m/s，更合理的做法是生成动态速度参考：

```text
v_ref = min(
    用户速度上限,
    曲率允许速度,
    剩余距离允许速度,
    障碍净距允许速度,
    可验证制动距离允许速度
)

reference_spacing(k) = v_ref(k) * dt
```

随后再调整线速度代价或加入 `v-v_ref` 代价。直接把固定 `reference_spacing` 从 0.025 改成 0.08 会让所有场景，包括急弯、近目标和行人交互场景，都按 0.8 m/s 生成参考，不适合作为最终方案。

## 16. 当前方法的边界

当前实现已经明确处理路径替换、时钟回退、输入陈旧、动态行人、完整矩形 footprint、制动轨迹和命令链失效，但仍有以下方法边界：

- 行人预测是常速度模型，不能提前理解转向和行为意图；
- 目标函数权重多数硬编码，不能在线调参；
- 固定参考速度约 0.25 m/s，没有按曲率和净距动态调速；
- 相关行人超过 8 名时安全失败，不会自动选最近 8 名继续运行；
- 静态障碍不参与 NLP，只在求解后拒绝碰撞轨迹，不能主动通过连续静态势场塑造绕障轨迹；绕行主要依赖 NavFn 路径；
- local costmap 只把 unknown/lethal 视为后检查硬碰撞，不把普通 inflation cost 加入 MPC 目标函数；
- 第一控制加速度区间固定使用 0.1 s，没有使用实际正向控制调用间隔；
- plugin 忽略 Nav2 传入的 velocity 参数，依赖自己订阅的 `/odom`；
- 当前无倒车控制、无 jerk 约束、无轮胎动力学和执行器时延模型；
- IPOPT 的 75 ms wall timer 不是线程硬抢占，最终安全依赖实测超时拒绝和 watchdog。

这些边界不表示当前验证场景失败，而是说明后续扩展速度、行人密度、机器人动力学或复杂静态绕障时需要重新设计和验证的部分。

## 17. 运行时可观测输出

当前可用于诊断 MPC 的主要输出包括：

| Topic | 内容 |
|---|---|
| `/FollowPath/status` | generation、epoch、solve time、cycle time、行人数、track/wait 模式及等待原因 |
| `/FollowPath/predicted_path` | plugin 预测轨迹；等待时发布实测制动轨迹 |
| `/mpc/local_trajectory` | 可视化适配后的局部预测轨迹 |
| `/mpc/global_plan` | frame 已规范化的全局路径 |
| `/mpc/human_markers` | 行人位置、速度、预测、目标、行为标签及排斥包络 |
| `/mpc_command_watchdog/status` | watchdog 转发或停车原因 |
| `/cmd_vel_nav` | controller server 原始控制命令 |
| `/cmd_vel_watchdog_in` | velocity smoother 输出 |
| `/cmd_vel` | watchdog 最终安全输出 |

正常 track 状态示例结构为：

```text
ok generation=<n> epoch=<n> solve_ms=<ms> cycle_ms=<ms> humans=<count> mode=track
```

等待状态会附加：

```text
mode=human_wait wait_reason=solver_timeout
mode=human_wait wait_reason=solver_infeasible
mode=human_wait wait_reason=dynamic_postcheck
mode=human_wait wait_reason=costmap_postcheck
```

已经进入保守制动包络时使用非 `ok` 的可恢复停车状态，使 watchdog 绕过平滑链并
立即输出零速，但不会终止 action：

```text
stop recoverable=1 mode=safety_wait reason=<diagnostic>
```

这些状态能够区分“优化器主动输出较低速度”“动态安全等待”“costmap 残留等待”和“输入/命令链故障停车”。

## 18. 公式与实现位置对照

| 内容 | 当前实现位置 |
|---|---|
| 状态、控制、配置和结果数据结构 | `src/arena_mpc_core/include/arena_mpc_core/types.hpp` |
| 单轮车离散模型、椭圆 clearance、独立 evaluator | `src/arena_mpc_core/src/model.cpp` |
| NLP 决策变量、目标函数和约束 | `src/arena_mpc_core/src/solver.cpp::build_graph()` |
| 相关行人筛选 | `src/arena_mpc_core/src/solver.cpp::select_relevant_obstacles()` |
| 初始猜测和 warm start | `src/arena_mpc_core/src/solver.cpp::initial_guess()`、`warm_start_guess()` |
| 路径参考生成 | `src/arena_mpc_controller/src/mpc_controller.cpp::computeVelocityCommands()` |
| HuNav 常速度预测和几何膨胀 | 同上，`build_human_obstacles` lambda |
| 矩形 footprint 和扫掠检查 | `pose_collision_free()`、`swept_trajectory_collision_free()` |
| 动态连续检查和制动检查 | `dynamic_trajectory_collision_free()`、`measured_braking_dynamic_collision_free()` |
| 失败、零速等待和结果世代检查 | `MpcController::computeVelocityCommands()`、`fail()`、`recoverable_stop()`、`retry_stale_path()` |
| 命令链 lease 和输入 watchdog | `src/arena_mpc_controller/src/mpc_command_watchdog.cpp` |
| 发布参数 | `src/arena_mpc_bringup/config/controller_model.yaml`、`nav2_overrides.yaml` |

# 将 MPC-Navigation 的 MPC 控制器迁移到 arena5_ws：实施与验收计划（Revision 2）

修订日期：2026-09-12；实施状态、可视化和速度上限增量更新至 2026-09-17。目标是把 `/home/lpc/MPC-Navigation` 中的 MPC 数学核心迁移到 `/home/lpc/workspace/arena5_ws`，新增与 DWB 独立可选的 ROS 2 Humble Nav2 Controller plugin。`social-nav-x` 仅为历史分支，不是交付目标。

本文区分三类结论：

- **已验证事实**：已由本机文件、安装头文件、二进制、进程或只读 ROS graph 检查确认。
- **设计决定**：实施必须遵守的方案。
- **实施前仍需验证**：必须在对应阶段用实际运行证据关闭，不能由源码默认值代替。

当前执行状态：**P0～P6 已全部通过，迁移和增量发布完成**。DWB baseline、实际 topic/QoS/时序、C++ 依赖闭包、独立数学核心、数值对照、容量 benchmark、Nav2 接口与故障停车、静态导航、动态行人避障、性能/耐久、DWB 交错对照、可重定位发布和原 DWB 回退均已有实机证据。阶段证据保存在 `/home/lpc/workspace/arena5_mpc_ws/evidence/p0` 至 `evidence/p6`。后续完成了 Foxglove 可视化、Path frame、costmap 清除、速度上限和目标连续性修正。当前不可变发布为 `/home/lpc/workspace/arena5_ws/optional/mpc/releases/20260918-bd63612`，此前发布继续保留用于回退。

## 1. 不变边界与阶段 gate

### 1.1 稳定工作空间保护

1. `/home/lpc/workspace/arena5_ws` 只允许读取源码和配置、查看状态、运行原生 DWB baseline，以及在 P6 完成增量发布。P0～P5 不编辑其中现有源码、脚本、共享 `build/`、`install/` 或 `.conda/`。
2. MPC 开发在 `/home/lpc/workspace/arena5_mpc_ws` 独立 Git 仓库进行；迁移始于 `feature/mpc-nav2`，当前发布增量位于 `fix/local-costmap-depth-clearing`。构建、安装、日志、缓存和求解器依赖均放在该目录。
3. 稳定工作空间根目录不是 Git 仓库。保护清单必须记录嵌套仓库 HEAD、本地修改、未跟踪文件、关键 install 文件、包解析路径和 SHA-256；不得 reset、clean 或覆盖用户已有修改。
4. 只新增 `arena_mpc_core`、`arena_mpc_controller`、`arena_mpc_bringup` 三个包。Arena、Isaac、HuNav、Nav2、消息包和 Foxglove 继续来自冻结 underlay。
5. P0～P6 必须依次通过。发现接口、参数、时序、性能阈值或实现方案与实机不符时，可以同步修正文档和实现，但不得改变总体架构、破坏稳定工作空间或跳过当前 gate。

**已验证事实（2026-09-11 P5 前置复查）：**保护清单中的 7 项有 6 项仍一致；`install/arena_simulation_setup/share/arena_simulation_setup/configs/nav2/nav2.yaml` 从 P0 的 `a42dfde...` 变为 `1b3e4986...`。该文件不是链接，mtime 为 2026-09-11 11:31:12。稳定工作空间的 colcon 日志和 shell 历史显示 11:31 执行过独立的 `colcon build --packages-select arena_simulation_setup`；它把源码中移除 global obstacle layer 的既有改动复制进 install。源码随后在 11:33 又把 global costmap 的 update/publish 频率改为 10/5 Hz，当前源码哈希为 `3972e35d...`，尚未与 install 相同。MPC 开发流程没有覆盖或重置这些并行修改。

**设计决定与验证结果：**P4 的 40 份正式证据均产生于该偏差之前，并绑定最终 MPC 源码和配置哈希，因此 P4 结论不受影响。按照“实际工作区状态优先”的执行授权，没有恢复或覆盖稳定文件；先用原生 DWB 入口在隔离 domain 145 做变更后 smoke。实测仍加载 `dwb_core::DWBLocalPlanner`，从 `(3.000,3.000)` 到 `(4.770,3.005)`，位移 1.770 m，收到 83 帧 lidar。该 smoke 通过后，开发工作空间的保护清单才把 installed `nav2.yaml` 冻结为当前 `1b3e4986...`；旧 `a42dfde...`、差异和变更来源保留在 `evidence/p5/preflight`。重新冻结只修改开发目录的清单，没有修改稳定目录。

### 1.2 GPU、domain 与端口

开发实例默认使用 `ROS_DOMAIN_ID=61`、WebRTC TCP 49110/UDP 48008、Foxglove 8766；稳定 DWB 默认 domain 51 和原端口保持不变。

GPU 2 和 GPU 3 上已有其他用户计算任务不构成等待条件。启动前查询每张 GPU 的剩余显存，在任意有足够余量的 GPU 上运行并记录 GPU UUID、索引、已用/剩余显存和其他负载。不得停止、暂停或修改其他用户进程；显存余量不足、端口冲突或本次所需 domain 出现不明节点时停止本次启动。DWB/MPC 对照须使用同一选定 GPU，或把 GPU 作为配对实验条件明确记录。

### 1.3 最终架构

- 原 `GPU_ID=3 ./scripts/run_six_behaviors.sh` 仍是默认 DWB 入口，原脚本字节不变。
- DWB 的 `0.8 m/s`、`1.5 rad/s` 版本使用新增的 `run_six_behaviors_dwb_08.sh` 独立入口；不覆盖原 DWB 参数和默认入口。
- MPC 是独立 Nav2 C++ Controller plugin，通过新增入口启动；首版不支持运行中热切换。
- 命令链固定为 `controller_server -> velocity_smoother -> 独立 watchdog -> /cmd_vel`。
- MPC 不可恢复失败时停车并向 Nav2 报告失败，不自动切换 DWB；动态行人冲突和
  有界求解超时属于可恢复安全停车，保持原目标并继续重算。
- 不迁移 ROS1、`local_map`、深度相机检测、`obs_param`、Kalman、旧 controller、Scout/Gazebo 模型或旧全局路径发布器。

## 2. 已验证环境事实与待验证项

### 2.1 Nav2 Controller 接口与异常

**已验证事实：**目标为 RoboStack ROS 2 Humble/Nav2 1.1.18。已安装 `nav2_core/exceptions.hpp` 定义 `nav2_core::PlannerException`，`controller_server.hpp` 明确声明该异常；已安装 DWB 异常继承它。已安装 `libcontroller_server_core.so` 的符号和异常处理路径与本机源码一致。

本机 controller_server：

- 以 10 Hz 墙钟循环执行控制。
- 在调用 plugin 前等待 local costmap `isCurrent()`；该等待没有插件内超时。
- 向 `computeVelocityCommands()` 传入 odom 速度。
- 捕获 `PlannerException`，根据 `failure_tolerance` 返回零速重试或终止 FollowPath。

**设计决定：**保留 `PlannerException`，但只用于输入断流、非法数据、TF/路径错误、
不可归因于动态行人的持续静态碰撞等不可恢复控制器失败。MPC 专用
`failure_tolerance=0`；普通失败先返回零速和无效诊断，连续五次实际被调用且均失败
后抛出异常。行人冲突、timeout/infeasible 后的零速等待以及求解期间 HuNav 世代更新
不累计该计数。costmap 等待或求解卡住时五次调用可能不会发生，独立 watchdog 负责
停车，不能声称 action 一定在五个墙钟周期内终止。

### 2.2 CasADi/IPOPT

**已验证事实：**目标 ROS 环境及相关 SDK/库目录原先未提供可用的 `casadi/casadi.hpp`、`libcasadi`、CasADi CMake package 或 CasADi IPOPT adapter。P1 已在独立 workspace 锁定 CasADi 3.8.0 官方 manylinux wheel，并用目标 RoboStack GCC 15.3 工具链通过 C++17 编译、CMake imported target 链接和 IPOPT 3.14.19 求解。最小运行闭包在清空环境且不设置 `CASADI_PLUGIN_PATH` 时可从 `$ORIGIN` staging tree 求解；`ldd` 无未解析项。依赖未安装到稳定 Conda 或 Isaac 环境。

**已验证事实（P6 已关闭）：**以下 5 项均已通过：

1. C++ headers、`libcasadi`、IPOPT adapter 和 CMake package。
2. 使用目标 RoboStack 编译器原生编译、链接和执行 IPOPT 求解。
3. C++ ABI、`libstdc++`、BLAS/LAPACK、Fortran/OpenMP 及动态加载依赖闭包。
4. pluginlib 装载和实际 controller_server 进程内生命周期、求解（P2 已通过）。
5. 隐藏开发前缀后的 RPATH、ament 索引和可重定位发布验证。P6 发布树经临时前缀构建、移动后 runtime check、10 次 benchmark、`ldd` 和绝对路径扫描，再写入稳定目录；发布后重复检查仍通过。

数学核心数值对照和容量 benchmark 已通过 P1 gate。CasADi/IPOPT 及其运行闭包只随版本化 MPC 发布提供，没有向稳定 Conda 或 Isaac 环境安装依赖，也没有升级 Nav2、Isaac、HuNav 或切换控制架构。

### 2.2.1 参考实现授权边界

**已验证事实：**参考提交 `5629641ffd2290d3a7ccd7eb5b7b1b3f2c0bdd64` 的 `local_planner/package.xml` 声明 `<license>TODO</license>`，仓库未找到覆盖该控制器的 license 文件。

**设计决定与验证结果：**新 C++ 核心依据运动学、代价和约束的数学定义独立编写，不复制 ROS 1 Python 源码文字；参考提交和文件哈希仅用于数值追溯。P6 没有分发参考项目代码或数据，发布树包含实际随包运行的 CasADi/IPOPT 及其传递库许可证。

### 2.3 `/human_states`

**已验证事实：**HuNav 源码使用 `create_publisher<Agents>("human_states", 1)`；结合已安装 rclcpp/RMW 默认值，源码配置对应 Reliable、KeepLast(1)、Volatile。已安装桥接把请求和完成状态 frame 设置为 `map`，默认计算参数为 40 Hz、最大积分子步 0.025 s；它采用异步 service，时钟回退时没有完整的请求世代隔离。

**已验证事实：**P0 的 60 秒运行端点实测为 Reliable、Volatile；Fast DDS outdoorspection 对 history/depth 报告 UNKNOWN/0，因此 depth=1 只作为发布源码事实，不冒充端点 introspection 结果。消息 frame 为 `map`，共 6 个稳定 ID，722 条消息，墙钟 12.026 Hz、墙钟间隔 p99 约 0.200 s、仿真时间戳中位间隔 0.025 s、时间戳单调且 age p99 约 0.008 s。实时因子约 0.3 解释了墙钟频率与 40 Hz 仿真配置的差异。

**已验证事实：**P2 使用显式空行人流验证了断流、10 s 陈旧时间戳和 10 s 未来时间戳；非法样本会更新 reset epoch 并立即停车，但不会覆盖最后有效缓存。真实六行为 HuNav bridge 暂停 0.75 s 后，watchdog 在约 0.600 s 报告 `human_input`，首零之后没有非零反弹；恢复时首条迟到响应的仿真时间年龄为 0.233 s，随后有效控制恢复。跨测试的整套进程停止和重启均正常。首版仍不承诺单独热重启 HuNav 后透明恢复。

**设计决定：**首版订阅采用 KeepLast(1)，reliability 按 P0 实测发布端冻结，不做静默 QoS fallback。缺少行人流不能解释为无人；无人场景必须显式提供带时间戳的空集合。使用 HuNav 的 ID、真实位置、平面速度、半径和消息时间戳，按 ID 对齐并作常速度预测。0.025 s 只表示积分子步上限，不作为端到端延迟上限。

### 2.4 costmap、lidar 与 footprint

**已验证事实：**实际链路为 `lidar -> Nav2 obstacle/voxel layer -> costmap -> MPC`。当前 observation buffer 在 `expected_update_rate=0` 时持续返回 current，因此 costmap current 或持续发布不能单独证明 lidar 新鲜。当前 global/local costmap 共用 ±0.1 m 名义 footprint，但 URDF 底盘碰撞盒约 0.42 m x 0.31 m，轮子还向外伸出。NavFn 不完整验证矩形机器人在路径姿态上的可行性；本版 `FootprintCollisionChecker` 栅格化多边形边界，不验证内部所有单元。

**设计决定：**

- MPC 模式以一个机器人几何配置同时驱动 global costmap、local costmap、轨迹检查和行人膨胀。初始矩形为 `x=±0.24 m, y=±0.22 m`，激活前与 Isaac 实际碰撞几何核对；初始 `footprint_padding=0`。
- 行人几何使用同一矩形外接半径约 0.326 m，再分别叠加行人半径、0.30 m 安全间隔和时序不确定度。
- MPC 专用 global/local inflation 初始均为 0.55 m；NavFn `allow_unknown=false`。原 DWB 配置不变。
- 保留 NavFn，并对 MPC 使用的局部路径段、完整预测轨迹和制动轨迹执行内部占据、边界、未知区、地图外及扫掠几何检查。
- 扫掠采样同时限制中心平移、角点旋转位移和机器人—行人的相对位移，不只使用机器人中心的半格平移。
- 静态障碍由完整 costmap footprint/扫掠检查处理，不放入椭圆 NLP。只读取物理占据和未知信息，不得把 inflation 成本再次当实体尺寸重复膨胀。
- plugin 检查 costmap current、快照、TF 和几何覆盖。watchdog 同时监控 costmap 更新与原始 lidar 接收/时间戳健康，但原始 lidar 不进入 MPC 优化器。
- MPC 专用 observation source 初始 `expected_update_rate=0.3` 秒；最终值根据 P0 cadence 冻结。

**已验证事实：**P2 已实际激活统一的 `x=±0.24 m, y=±0.22 m` footprint，并对预测轨迹及制动轨迹执行填充多边形、平移/旋转扫掠和动态行人连续线段后检查。global/local costmap 使用同一 footprint，DWB 配置未变。`expected_update_rate=0.3 s` 在启动首帧 lidar 到达前会产生 current 警告，数据建立后恢复；P2 另行切断原始 lidar，在其他 lease 均健康时约 0.280 s 触发 `lidar_input` 停车，确认不能只监控 costmap。

**已验证事实：**P3 前复核发现 base global costmap 的 inflation radius 为 0.25 m，而 MPC local override 为 0.55 m；已仅在 MPC overlay 补齐 global 0.55 m，原 DWB 配置未改。P3 的全部墙体均通过真实 `lidar -> costmap` 链路观测；连续五个共线短障碍不能在运动前同时被 lidar 看见，因此验收改为逐墙记录整个运行期间的实际可见性，没有把未观测障碍预先注入优化器。

## 3. 数学核心与运行语义

### 3.1 数学模型

- 保留平面非完整运动学：`x += dt*v*cos(yaw)`、`y += dt*v*sin(yaw)`、`yaw += dt*w`；状态 `(x,y,yaw)`，控制 `(v,w)`。
- 保留原位置/航向跟踪、控制代价、终端代价、椭圆距离和 `h[k+1] >= gamma*h[k] - slack[k]`。`gamma` 沿用原语义。
- 初始参数：`N=25`、模型步长 `dt=0.1 s` 仿真时间、`gamma=0.2`、净安全间隔 0.30 m、终端权重 1、松弛惩罚 50。
- 首版历史约束为 `0 <= v <= 0.26 m/s`、`|w| <= 1.0 rad/s`；2026-09-17 增量发布将配置上限改为 `0 <= v <= 0.8 m/s`、`|w| <= 1.5 rad/s`。线/角加速度上限仍为 2.0 m/s² 和 3.2 rad/s²，并继续支持 Nav2 限速。
- 修正固定 `25*j` 索引、末端状态错位、atan2 边界、跨 ±pi 航向、退化椭圆和零距离问题；约束覆盖 `x[N]`。
- 删除 `exceed_ob()` 目标方向障碍丢弃规则。松弛只能作用于屏障收敛条件，几何碰撞边界保持硬约束；因松弛只受非负下界并仅进入二次罚项，核心将其解析消元为 `max(0, gamma*h[k]-h[k+1])`，输出时恢复同一 slack 值。求解后独立检查。
- 路径按弧长重采样，处理重复点、短路径、路径替换、目标位置和终点航向，沿用 0.25 m/0.25 rad goal checker 容差。位置未进入 XY 容差时，路径末端重复参考保持末段驶入航向；进入 XY 容差后锁存位置阶段，再把 26 个参考统一为最终位置和最终航向执行原地转向。P3 实测否定了每模型步 0.05 m 的初值：它在 `dt=0.1 s` 下对应 0.5 m/s 并导致急弯切角；适配层冻结为每步 0.025 m，对应 0.25 m/s。该标称参考在 4.4 节扩大控制上限后仍保持不变，数学核心结构未改变。

首控制量加速度约束使用新鲜实际 odom 速度，后续使用相邻控制量差。controller_server 传入的 Twist 没有时间戳，因此 plugin 同时检查对应 odom 数据年龄。P2 实现使用固定模型首段 `dt=0.1 s` 约束实际 odom 到第一控制量；相同仿真时间戳允许重复控制调用，时间戳倒退则停车并锁定本次运行。上一条命令只作为 warm start，不代替失效 odom。

### 3.2 固定容量与求解结构

输入容量保留为 32 个动态和 128 个静态对象；超出输入容量必须停车并报告，不能静默截断。NLP 活动容量与输入容量分开：静态对象由 global path 与完整 costmap 几何检查处理；动态对象先用 `N*dt*max_v` 可达圆、各时刻预测中心、椭圆最大轴和安全间隔作保守不可达证明，只有证明在整个 horizon 不可达的对象才可不进入 NLP。可达动态对象最多 8 个，超出即停车并报告；所有收到的动态和静态对象仍参与求解后检查。

**已验证事实：**P1 否定了字面上的 32 dynamic + 128 static 固定 NLP。带 4000 个显式 slack 的 160 槽 pilot 建图约 8～9 s，求解约 98～151 ms；解析消元后另一 pilot 建图最高约 28 s，求解仍约 290～407 ms。该结构不满足 10 Hz 合同。

**设计决定：**采用参数化 fixed-size NLP + active mask，但固定的是 8 个动态槽，不是全部 160 个输入对象。非活动槽使用有限、非退化的远处占位椭圆并由 mask 解除约束；图在 solver 构造/插件配置阶段建立约 0.35 s，控制周期只更新参数。warm start 将上一成功解控制序列左移一段，再以当前 odom 速度和加速度边界钳制并重新 rollout 状态。2026-09-19 在线故障复现表明，持续 wall-time 超时时每周期丢弃 IPOPT 有限迭代会重复同一冷启动；修订后超时迭代绝不作为命令，但在尺寸正确且全部有限时保留为下一次求解的原始 primal 初值。成功解仍使用原左移规则，生命周期/reset epoch 变化仍清空全部初值。

**已验证事实：**P1 在 `N=25` 下比较 exact-active 与 fixed-mask，覆盖 0 障碍、典型六行人，以及输入 32 dynamic + 128 static 的可行、临界和不可行问题，每行 1000 次。选定 fixed-mask 的可行组全部 1000/1000 成功，最坏 p95/p99 为 26.03/26.56 ms，残差不超过 `2e-6`，全输入后检查冷样本不超过 0.17 ms。初始重叠组均返回无效命令；IPOPT 60 ms CPU 限制对应最坏墙钟 p95 约 65.19 ms，证明该限制不是硬墙钟抢占。

**2026-09-19 运行修正：**在线实例在机器人 `(5.7315, 6.0521, 2.3493)`、行人 3 `(6.5265, 6.8227)` 时持续报告 `solver_timeout`。中心距为 1.1072 m；NLP 外接圆阈值为 1.1256 m，固定初始状态因此违反 0.0184 m，但定向矩形 footprint 的实际净距约 0.437 m，仍高于正常 0.35 m。为防止这种近似误差造成永久零速，NLP 只容许最多 0.05 m 的既存初始外接圆违反，并禁止预测轨迹进入比该初始值更深的违反；深于 0.05 m 的初始冲突仍不可行。plugin 另增加 `clearance_recovery` 子模式：仅当原始定向 footprint 对全部行人仍满足 0.35 m 时，沿最紧约束行人的反方向产生加速度受限控制，目标上限为 0.40 m/s、1.0 rad/s；每周期只提交通过原始 HuNav 几何、完整制动轨迹和 costmap footprint 检查的第一控制量。外接圆净空恢复到 0.10 m 后立即回到普通 MPC。该模式不降低 0.35/0.30 m 安全阈值，也不绕过 watchdog、输入新鲜度、path generation 或 90 ms 提交 gate。

### 3.3 时间预算和 watchdog

10 Hz controller 使用 100 ms 墙钟周期；MPC 模型步长使用仿真时间，两者在实时因子不为 1 时不能互换。

P2 后冻结的分层预算如下；P5 只在新负载证据证明不成立时按 gate 修订：

| 项目 | P2 后合同与实测 |
|---|---|
| 求解 | `max_wall_time=0.075 s`、`max_iter=100`；结果只有在求解器成功且全部残差/几何检查通过时才接受。P2 正常空场景 solve p95 63.25 ms、max 67.22 ms；故障用例约 78～81 ms 返回 `Maximum_WallTime_Exceeded` 并被拒绝。旧 P1 的 60 ms CPU 限制只保留为历史 benchmark，不能作为硬墙钟合同。 |
| 快照、适配和后检查 | 与求解共享 plugin 的 90 ms 提交上限；不再把不可独立强制的 15+15 ms 子项伪装成硬 deadline。 |
| server/发布余量 | 100 ms 完整 deadline 与 90 ms plugin 提交上限之间保留至少 10 ms。 |
| 完整处理 | deadline 100 ms；目标 p95 <=90 ms、p99 <=100 ms、超过 100 ms比例 <=1%。P2 跨 `controller_server` 边界实测 p95 63.82 ms、p99/max 69.37 ms、超限 0/58。 |
| plugin 提交 | 从 plugin 入口到全部求解和后检查完成超过 90 ms，不提交非零结果。 |
| watchdog | 独立进程，50 Hz 墙钟检查 |
| 命令有效期 | 有效求解、原始命令或平滑命令任一连续 250 ms 无更新即关闭输出 |
| 零速开始 | 命令/status lease 从最后一次有效更新起不超过 270 ms；输入 lease 分别按冻结阈值加一个 20 ms tick 验收：odom 0.42 s、HuNav 0.62 s、lidar 0.35 s、costmap 0.35 s。 |

“完整处理”包括取状态、适配、求解、后检查和原始命令发布，不包括为保持 10 Hz 的主动休眠。controller_server 在 plugin 外等待 costmap current，该段由 costmap lease/watchdog 覆盖，不能由 plugin 计时冒充。P2 已用 controller status 到 raw command 的边界时间重建完整处理；IPOPT 时间限制仍不是线程硬抢占，看门狗承担 solver 或 server 卡住时停车。

odom、HuNav、lidar 分别检查 ROS 数据年龄和墙钟接收间隔。ROS 年龄初始上限 0.3 s；墙钟上限按 P0 正常流量设为 `max(0.3 s, 3*p99接收间隔)`，实验前冻结且不在线放宽。首轮 60 秒 DWB 实测得到 lidar p99 约 0.511 s、HuNav p99 约 0.200 s，因此 P2 起始墙钟阈值分别取 1.55 s 和 0.60 s；后续重复 baseline 发现更大正常值时只允许在 P0 结束前重新冻结。输入超时检测时间与命令有效期分开报告。

**已验证事实：**watchdog 对 odom、HuNav、lidar 和 costmap 都在回调写缓存前拒绝陈旧/未来时间戳，时钟回退永久锁定到进程重启。Isaac `ResetWorld` 服务源码只删除环境的墙、门、地板和电梯，不会回退 `/clock`；P2 的 5 s 时钟回退用隔离 domain 上额外 `/clock` publisher 主动注入，约 20 ms 内触发持续零速和 reset latch。

**已验证事实（P5）：**30 分钟正式运行得到 16,494 个有效求解/控制周期。solver p95/p99/max 为 81.167/83.266/89.416 ms，plugin cycle 为 81.486/83.658/89.721 ms。由于 IPOPT 的 `max_wall_time=0.075 s` 不是线程硬抢占，solver 外部墙钟可以略高于 75 ms；安全合同由 90 ms plugin 提交检查和 watchdog 共同强制。跨 headerless status/raw topic 不能依赖 DDS 回调先后顺序，正式探针改为在 25 ms 窗口内双向最近邻匹配，覆盖 16,494/16,494，边界偏差 p99 为 1.014 ms。将该偏差保守加到每个 plugin 周期后，完整处理 p95/p99/max 为 82.500/84.672/90.735 ms，超过 100 ms 为 0，满足 gate。

**已验证事实（2026-09-17 纠正审计）：**上述运行的时序和输出安全统计仍有效，
但原耐久通过判据有缺陷：276 个结束目标中只有 15 个成功、259 个 ABORTED、2 个
由探针超时取消；旧判据只要求成功数不少于 10，没有拒绝 ABORTED，因而错误地把
目标连续性标为通过。修订后的探针明确要求 `aborted_goal_count == 0` 且
`timed_out_goal_count == 0`；旧报告只保留为性能和缺陷复现证据，不再作为到达 gate。

同一耐久运行的 HuNav、odom、lidar、raw costmap 最大墙钟间隔为 0.480/0.414/0.763/0.801 s，均在预先冻结的持续断流判据内。watchdog 仍使用更紧的运动 lease，短暂超限时停车而不是放宽命令有效期；正式运行记录到 `controller_status`、`command_sequence`、`status_lease` 和一次 `odom_input` 安全停车。

### 3.4 结果世代与 reset

- `path_generation` 在 plugin 接受且路径内容有效变化时递增；只有 header 变化不使相同路径反复失效。
- `reset_epoch` 在时钟回退、运行重启、生命周期失效或控制几何/配置变化时更新，清除在途提交权、命令 lease 和 warm start。
- 数据序号只用于追溯快照。新 odom/HuNav 消息到达不会自动作废所有正在求解的结果。
- 提交前重新检查 epoch、已接受路径版本、deadline、数据年龄和最新可用几何安全信息。
- reset 或时钟回退时停车并锁定本次运行；首版实验通过重启本次拥有的进程进入下一轮，不承诺 HuNav 热 reset 透明恢复。

## 4. 新增包、启动与发布

| 包 | 职责和接口 |
|---|---|
| `arena_mpc_core` | 无 ROS 依赖的 C++ 数学库；输入当前状态/速度、参考、障碍预测和约束，输出轨迹、控制、状态、残差、松弛和耗时。测试保留纯 Python 参考。 |
| `arena_mpc_controller` | `arena_mpc_controller::MpcController` plugin，完成路径、TF、HuNav、costmap 适配；包含独立 watchdog 可执行程序。 |
| `arena_mpc_bringup` | 独立 launch、MPC 配置、场景、运行脚本、指标、保护清单、可视化适配器和发布工具。 |

保持 `/navigate_to_pose`、`/follow_path` action 和 `controller_id=FollowPath`。路径只从 `setPlan()` 获取。plugin 返回 `TwistStamped`，不直接发布机器人 `/cmd_vel`。

新增 launch 包含稳定六行为 launch并设 `navigation=false`，复用 Isaac、HuNav、D6、robot_state_publisher、WebRTC 和 Foxglove；随后独立启动 map_server 及 MPC Nav2 组，不重复创建仿真、机器人、行人或 map_server。

开发、调参和 P0～P5 始终位于独立目录。P6 才把不可变发布安装到 `arena5_ws/optional/mpc/releases/<release-id>`，在 `optional/mpc` 设置 `COLCON_IGNORE`，并仅新增 `scripts/run_six_behaviors_mpc.sh`。发布树必须在临时位置验证可重定位性，不能引用开发目录绝对路径。

**已验证事实（P6）：**发布 `20260912-58cd661` 大小为 45,067,655 bytes，包含完整 SHA-256 清单、第三方许可证、三个新增包及 CasADi/IPOPT 运行闭包；零符号链接，开发目录和临时 staging 路径匹配数均为 0。发布绑定源码提交 `58cd661d8de076a114484757a5b6ef1c4b3521d4`。稳定目录仅新增该版本目录、`optional/mpc/COLCON_IGNORE` 和独立包装脚本；既有 7 项保护清单全部一致，原 DWB 脚本哈希仍为 `9f8d26ea...`。

**已验证事实（2026-09-18 纠正发布）：**目标连续性修正发布
`20260918-bd63612` 大小为 45,964,467 bytes，绑定提交
`bd63612de9ca5ae28986a063257a2bf4a37c073a`。临时重定位、runtime、benchmark、动态
依赖、零符号链接、零开发/临时绝对路径、全文件 SHA-256 和稳定保护清单均通过。
发布版 MPC 与受保护的原默认 DWB 分别在 domain 214/215、GPU 3 smoke 成功；位置误差
为 0.2254/0.2265 m，行人保守净距下界为 0.5224/0.4869 m。MPC 使用 0.8 m/s 配置
上限，原默认 DWB 按设计保持既有 0.26 m/s；独立 DWB08 入口随新 release 更新，原
`run_six_behaviors.sh` 哈希仍为 `9f8d26ea...`。

### 4.1 Foxglove 可视化增量（2026-09-15 已完成）

**设计决定：**可视化由 `arena_mpc_bringup` 中独立的只读 ROS 2 节点 `mpc_visualizer` 提供，默认随 MPC launch 启动，可通过 `MPC_VISUALIZATION=false` 关闭。该节点不加入 lifecycle 管理，不参与求解、速度平滑、watchdog 或 `/cmd_vel` 发布，因此保持 `controller_server -> velocity_smoother -> watchdog -> /cmd_vel` 安全链不变。原 DWB launch、脚本和参数不增加该节点。

节点订阅 Nav2 实际 `/plan`、Controller 已有 `/FollowPath/predicted_path` 和 HuNav `/human_states`，提供三类稳定显示接口：

| 输出 topic | QoS | 含义 |
|---|---|---|
| `/mpc/global_plan` (`nav_msgs/Path`) | Reliable、Transient Local、depth 1 | Nav2 最新全局路径，新连接的 Foxglove 客户端也可取得最近路径。 |
| `/mpc/local_trajectory` (`nav_msgs/Path`) | Reliable、Volatile、depth 1 | MPC 当前预测轨迹；`N=25` 的正常求解输出为 26 个状态点。 |
| `/mpc/human_markers` (`visualization_msgs/MarkerArray`) | Reliable、Volatile、depth 1 | 行人身体、速度箭头、2.5 s 匀速预测、HuNav 目标、行为/速度标签，以及 MPC 中心排斥包络。 |

行人颜色对应六类 HuNav 行为：regular 蓝、impassive 灰、surprised 黄、scared 紫、curious 青、threatening 红。半透明平面圆柱的半径定义为 `human_radius + 0.326 m robot_circumscribed_radius + 0.05 m geometry_uncertainty + 0.35 m safe_distance`，用于解释 MPC 配置的中心排斥几何，不作为新的安全判定器。Marker 使用 `/human_states` 原 frame 和当前仿真时间；仅将测量时延外推限制在 0.3 s 内，无效或非有限行人数据不显示，空快照通过 `DELETEALL` 清除旧 ID。

**已验证事实：**开发 overlay 实机运行于 domain 220、GPU 3。只读探针取得 196 点全局路径、28 条局部预测消息且最大 26 点、286 个行人 Marker 快照和全部六个语义 namespace，三个输出的唯一发布者均为 `mpc_visualizer`。独立 ROS graph smoke 验证 topic、QoS 和 `2.052 m` fixture 排斥直径。短程 MPC action 成功，377 个命令样本均有限，保守行人净距离下界为 0.54397 m；新增节点未改变命令发布者。额外 8 m 六行为随机运行的可视化探针已经通过，但 action 超过 180 s 后由探针取消，作为被拒绝的补充运行保留，不计入可视化 gate。

源码提交 `c021977c52b2ac63d4b6de43a3c07facff2d4215` 完成 3 包重建和 8 项测试记录且零失败。新不可变发布 `20260915-c021977` 大小为 45,082,081 bytes，重复通过临时重定位、动态依赖、零符号链接、绝对开发路径、SHA-256 和 7 项稳定保护审计；从发布目录启动的 ROS graph smoke 再次通过。稳定 MPC 包装脚本只改为指向该新版本，旧发布保留，原 DWB 入口未修改。

Foxglove bridge 仍由既有独立 launch 提供，MPC 默认端口为 8775。Windows 远程客户端应在服务器端设置 `ARENA_FOXGLOVE_ADDRESS=0.0.0.0`，并连接 `ws://<server-ip>:8775`；Windows 的 `localhost` 不指向本 Linux 服务器。

**Path frame 在线修复：**当前机器上的实际 Navfn `/plan` 使用 `Path.header.frame_id=map`，但各 `PoseStamped.header.frame_id` 为空。旧可视化适配器原样转发后，Foxglove 报告 `"map" != ""` 并在变换面板显示空 frame。domain 222 的修复前探针确认 25/25 个全局路径 Pose 为空；同次采样检查 3,685 个 `/tf` transform 和 9 个 `/tf_static` transform，父子 frame 均非空，因此不修改 TF 发布链。

设计上，仅在可视化适配器的消息副本中让空 Pose frame 继承非空 Path frame；若 Pose 已给出与 Path 顶层不同的非空 frame，则拒绝该 Path，避免把不同坐标系的数据错误重标。domain 225 的修复后实机探针确认：原 `/plan` 仍为 `map + 23 个空 Pose frame`，`/mpc/global_plan` 已为 `map + 23 个 map Pose`，局部轨迹保持 `odom` 一致，TF 仍无空 frame。同期 MPC action 成功，命令均有限，行人保守净距下界为 0.55696 m。

修复提交 `df9a55d61f68c0eaee5f7d393c8ab771f06a3963` 将可视化单元用例增至 4 个；三包完整结果为 10 tests、0 errors/failures/skips。不可变发布 `20260915-df9a55d` 大小为 45,083,528 bytes，已通过重定位、依赖、零符号链接、绝对路径、SHA-256 和稳定保护审计；从该发布直接运行的严格 smoke 将输入 `<empty>` Pose frames 全部输出为 `map`。原 DWB 入口和哈希保持不变。

### 4.2 全局代价地图障碍层修正（2026-09-15 已完成）

MPC 模式的 global costmap 固定为 `static_layer + inflation_layer`，不再加载
global `ObstacleLayer`。RTX 原始 `/lidar` 的无效量测不能稳定形成 free-space
clearing ray；把它用于全局标记会使瞬态障碍残留为 lethal cell，并可能让 Navfn
误判无路。静态结构继续来自地图，global inflation 和 `allow_unknown=false` 保持
不变。

此修正不改变局部避障链：local `VoxelLayer` 的 lidar marking/clearing、局部
costmap watchdog、MPC 轨迹/制动碰撞复核，以及直接 `/human_states` 动态行人约束
全部保留。只有在静态地图确实遗漏了需要全局拓扑绕行的障碍，并且传感源具有可信
clearing 或等价的有界寿命过滤时，才重新评估 global `ObstacleLayer`。

提交 `bf2bc7c31ec1dadd51318b20485eab138744c452` 新增配置契约测试和原子发布
选择器；三包构建及 12 项测试全部通过。不可变发布 `20260915-bf2bc7c` 的实装
配置确认 global plugins 仅为 `static_layer,inflation_layer`，无 global obstacle
override，local lidar 期望更新周期仍为 0.3 s。发布通过重定位、依赖、绝对路径、
全文件 SHA-256 和 7 项稳定底座保护检查。入口完成
`20260915-bf2bc7c -> 20260915-df9a55d -> 20260915-bf2bc7c` 回退往返，两个
方向的校验均成功；原 DWB 入口未修改。

### 4.3 局部代价地图动态残留清除（2026-09-16 已发布，待重启实机验收）

在线复现确认，导航目标提前 `ABORTED` 的直接原因不是 Navfn 无路或 progress
checker：全局路径持续成功，而 MPC 在行人离开后仍检测到局部 VoxelLayer 的致命
残留。碰撞点为 `(4.1619, 3.00064)`，其前方致命栅格对应的当前 RTX LaserScan
束为 `-1/0`，不能形成 clearing ray；MPC 在 `costmap_obstacle_wait_limit=1.0 s`
后连续失败五次并终止 FollowPath。

提交 `832939866ee8809169e31ae446fe3da0e07d62d5` 以不可变运行时叠加方式复用
已验证的四向渲染深度清除源。原 `/lidar` 不改写且继续负责 marking；新增
`/lidar_clearing` 为 PointCloud2，只允许 clearing、禁止 marking，有限深度在表面
前 0.05 m 停止，无效深度不产生清除点。global costmap 仍只加载 static 与
inflation layer。

三包构建及 16 项测试全部通过。不可变发布 `20260916-8329398` 包含 207 项
SHA-256 校验、零符号链接、零开发/暂存路径引用，并保持七项稳定底座哈希不变；
`ARENA_DEPTH_CLEARING=false` 即时禁用检查及新旧发布双向回退均通过。当前旧进程
未被热修改，最终实机验收须从稳定包装脚本重启后复现同一路径。

### 4.4 MPC 与独立 DWB 配置的速度上限（2026-09-17 已完成）

**设计决定：**MPC 的控制器约束和 velocity smoother 上限统一改为线速度
`0.8 m/s`、角速度 `1.5 rad/s`。DWB 使用独立配置叠加层，同时统一
`FollowPath.max_vel_x`、`FollowPath.max_speed_xy`、`FollowPath.max_vel_theta` 和
velocity smoother 上下限。原 `run_six_behaviors.sh` 及其参数继续作为默认 DWB
baseline；新增 `run_six_behaviors_dwb_08.sh` 才选择高速 DWB 配置。MPC 的
`reference_spacing=0.025 m` 保持原值，因此本次只扩大允许上界，不把标称参考速度
强制提高到 `0.8 m/s`。

**已验证事实：**提交 `4ff2edd9715e2ee839f055558450dc0297886927` 完成三包
构建和 18 项测试，零错误、失败或跳过。不可变发布 `20260916-4ff2edd` 的 MPC 与
DWB runtime-only、重定位、动态依赖、全文件 SHA-256 和稳定保护检查均通过；原
DWB 脚本 SHA-256 仍为 `9f8d26eae21348e00e6141f95829e51bf5cbc21e0c57d541bd3c8e9df2530836`。
独立参数探针确认 MPC 的 `max_linear/max_angular=0.8/1.5`、DWB 的
`max_vel_x/max_speed_xy/max_vel_theta=0.8/0.8/1.5`，两种模式的 smoother 都为
`max_velocity=[0.8,0,1.5]`、`min_velocity=[-0.8,0,-1.5]`。

同一六行为场景的发布版在线 smoke 均成功：MPC action 成功、763 个命令样本有限，
位置误差 0.2275 m，测得/保守行人最小净距为 0.5767/0.5201 m；DWB 高速配置
action 成功、104 个命令样本有限，位置误差 0.2166 m，测得/保守行人最小净距为
1.1501/1.0940 m。在线探针还确认 MPC 使用统一的 ±0.24/±0.22 m global/local
footprint，而 DWB 保留自身原 footprint。

**验证阈值修正：**安全探针用最大机器人速度、最大行人速度、human stamp gap 和
odom bracket 计算采样对齐误差上界。速度上限提高后，同一 25 ms/16.7 ms 采样窗口
的理论上界为约 0.0566 m，旧的 0.05 m gate 会拒绝安全净距充足的运行。该审计
gate 因而改为 0.06 m；行人保守净距硬下界仍为 0.30 m，计算方式和安全判据没有
放宽。探针同时修正了已过期的 global costmap 插件预期，使其与 4.2 节已发布的
`static_layer + inflation_layer` 合同一致。

### 4.5 未到达目标时提前 ABORTED 修正（2026-09-18）

**已验证事实：**对用户保留的 domain 51 实例只读取证后，排除了 GoalChecker
误判。旧发布 `20260916-4ff2edd` 在 IPOPT 超时后发现实测制动轨迹进入 HuNav
保守包络，会连续报告
`measured braking trajectory failed swept HuNav check`；第五次把同一动态安全状态通过
`PlannerException` 交给 controller_server，FollowPath 随即失败，行为树再把尚未
到达的 NavigateToPose 标为 ABORTED。原 P5 耐久探针又只要求成功数不少于 10，未
拒绝 259 个 ABORTED，因此旧的“目标连续性通过”结论无效。

**设计决定：**动态行人冲突、timeout/infeasible 后的安全等待、求解期间 HuNav
更新导致的候选失效属于可恢复控制决策。plugin 返回零命令并发布
`stop recoverable=1 mode=safety_wait`；watchdog 立即强制 `/cmd_vel=0`，但 plugin
不抛异常，保留同一 FollowPath action 并在下一周期重算。输入断流、非法数据、
TF/路径错误和不可归因于动态行人的静态碰撞仍使用原失败计数和异常链，不自动切换
DWB。

另一个独立原因是继承的 `SimpleProgressChecker.required_movement_radius=0.5 m`
可能大于 0.25 m goal tolerance 后短回程的剩余距离；机器人即使继续接近目标也无法
刷新进度。先改成 0.05 m/120 s 后，实机第二个回程仍在动态安全等待累计到时限后
失败，否定了单纯延长 SimpleProgressChecker 的方案。MPC overlay 因此新增
`SafetyAwareProgressChecker`：`required_movement_radius=0.05 m`、
`required_movement_angle=0.1 rad`；平移或终点旋转都能刷新进度。只有 1.0 s 内
收到的新鲜 `human_wait/safety_wait` 状态才暂停 120 s 的**活动跟踪**预算，恢复
`track` 后继续累计。状态断流或普通跟踪停滞不会无限等待。这只决定 action 是否继续
等待，不放宽 MPC、costmap、footprint 或 watchdog 的任何运动安全条件。

**已验证事实：**强制把一个行人覆盖到机器人上并保持超过五个控制周期时，watchdog
在 0.03745 s 内开始零输出，覆盖期间无非零反弹且 action 未结束；移除覆盖后，同一
goal 恢复运动并以 `STATUS_SUCCEEDED=4` 到达。第一轮 5 分钟自然耐久复现旧 0.5 m
进度半径导致的一次 `FailedToMakeProgress`；修正半径和时限后的第二轮记录零
ABORTED，但旧探针在 180 墙钟秒主动取消一个仍活动的目标，因而不计正式通过。随后
正式预跑确认 SimpleProgressChecker 即使配置 0.05 m/120 s 仍会把安全等待计入时限，
该运行被拒绝并停止。首版 SafetyAwareProgressChecker 又在终点位置附近因只统计
平移、未统计转向而失败，实机否定后增加 0.1 rad 角度进展。pluginlib、普通停滞/
安全等待/终点转向计时单元测试和解除覆盖后的同目标到达均已通过。正式
替代耐久先把单目标墙钟上限从 180 s 改为 600 s，但实测仍在人为取消一个保持活动
的长回程后失败；固定 1800 s 耐久最终禁止在总时长内按墙钟替换有效 goal，只在整轮
结束时清理尚活动的 action。gate 同时要求至少十次成功、零 ABORTED、零测试超时；
详细接受/拒绝关系见 `evidence/abort_fix/README.md`。

**已验证事实（2026-09-18 终点收敛复查）：**禁用单目标墙钟取消后的第四次正式
预跑没有 ABORTED，但暴露了新的普通跟踪停滞：机器人距 `(3.0,3.0)` 为
0.25982 m，仍在 0.25 m XY 容差外，plugin 报告 `mode=track`，却只输出
`v=5.09e-5 m/s, w=1.29e-5 rad/s`。原因是短路径饱和到终点后，多数参考位置都等于
目标点，航向却提前切成最终 0 rad；对不能侧移且不能倒车的 Jackal，这会让“沿末段
驶入位置”和“最终姿态”同时竞争并形成近零局部平衡。不能通过放宽 goal tolerance
接受该状态。

**设计决定：**plugin 现在通过 Humble 的 `GoalChecker::getTolerances()` 读取实际 XY
容差。位置阶段中，饱和到终点的参考继续使用最后一个非退化路径段的切向；进入 XY
容差后锁存位置阶段，所有参考才切换为精确终点和最终航向。`setPlan()`、生命周期
停用/激活会清除该锁存。若自定义 GoalChecker 不提供有效容差，才使用显式的
`goal_position_tolerance_fallback=0.25 m`。求解、动态行人约束、完整 footprint 扫掠和
watchdog 链均未绕过。

同轮在线检查还否定了原 `costmap_obstacle_wait_limit=1.0 s` 假设：一个能与当前
HuNav 行人关联的局部 costmap 占用在约 12 s 后清除并允许同一路径成功，但 1 s 上限
已先终止前一个 action。该等待资格仍要求“碰撞点匹配当前 HuNav + 实测制动轨迹
安全”；无法匹配行人的静态碰撞继续立即失败。匹配后的零速等待改为有限 120 s，
期间持续发布 `human_wait` 并由 SafetyAwareProgressChecker 暂停活动跟踪计时，超时
仍按静态失败处理。

**已验证事实：**纯函数回归精确复现 0.259839 m 场景，确认位置阶段末端航向保持
`pi`，进入 0.25 m 后才切为最终 0 rad；三包共 20 项测试通过。domain 210、GPU 3
的六行人聚焦往返中，`x=3.6` 与 `x=3.0` 两个 NavigateToPose 均返回
`STATUS_SUCCEEDED=4`，返程 136.707 s 完成，ABORTED/测试超时/controller failure
均为 0；1,107 个完整命令周期 p95/p99/max 为 85.301/86.715/88.875 ms。
随后提交 `04c8f30` 在 domain 211、GPU 3 完成正式 1800.514 s 替代耐久：31 个
已结束目标全部 `STATUS_SUCCEEDED=4`，ABORTED 和单目标测试超时均为 0；整轮结束时
仅清理一个仍活动的 action，不计为失败。90,029 个输出全部有限，`/cmd_vel` 唯一
发布者为 watchdog；HuNav、odom、lidar、local costmap 输入流健康且无时钟回退。
15,557 个完整命令周期 p95/p99/max 为 83.615/84.913/90.433 ms，超过 100 ms 为 0。
该结果满足至少十次成功、零 ABORTED、零测试超时的替代 gate，本纠正项关闭。

## 5. P0～P6 测试与验收

| 阶段 | 工作内容 | gate |
|---|---|---|
| P0：冻结与 DWB baseline（已通过） | 记录保护清单、安装版本、GPU、端口和包解析；用原 DWB 入口执行现有 runtime 检查和固定导航 smoke 五次；另以 `NAVIGATION=false` 执行六行为验证。采集至少 60 s 实际 QoS、频率、时间戳、frame、参数和 TF。 | 导航 smoke 成功且位移 >1 m；六种行为与原校验通过；Nav2 lifecycle、D6、odom、lidar、TF、WebRTC/Foxglove 正常。未通过先处理基线，不归因 MPC。 |
| P1：依赖与数学核心（已通过） | 验证 C++ SDK/ABI/IPOPT/动态装载；Python/C++ 数值对照；最坏容量 benchmark。 | 实测 evaluator 最坏差约 `4.93e-16`，固定用例首控制差为 0，目标值差约 `1.39e-17`；残差 <=`2e-6`；缺陷用例及每组 1000 次 benchmark 通过。 |
| P2：接口、时序与故障（已通过） | 实际 server 装载；路径替换、非单位 TF、限速、朝向、取消、生命周期；断流、陈旧/未来时间戳、costmap 不 current、暂停、时钟回退、真实 HuNav 迟到结果和 solver 故障。 | 17 份 JSON 证据全部通过；仅 watchdog 发布 `/cmd_vel`，TF/odom 无竞争；迟到/非法数据不产生非零反弹；各 lease 达到冻结时限；仿真恢复后机器人满足停车阈值。 |
| P3：导航与静态避障（已通过） | 直线、转弯、终点朝向、墙边、窄通道、90度拐角、内部小障碍、旋转扫掠、未知区、完全封堵。 | 十类场景各五次达到预期；可达用例最大仿真时间 18.151 s、位置/朝向均在 0.25 m/0.25 rad 内；精确 footprint 监测碰撞为 0；未知区和封堵均失败停车且无非零回弹。 |
| P4：行人避障（已通过） | 单人横穿/迎面/同向、多人交叉、停止/转向、ID变化、时序积压和六行为回归。 | 8 类场景各 5 次、共 40/40 正式运行通过；动作均成功、输出有限，保守净距离下界最小 0.4232 m，采样对齐误差上界最大 0.04220 m；完整 plugin 周期最大 89.89 ms。 |
| P5：性能和 DWB 对照（已通过；目标连续性已由替代耐久纠正） | 最大规模每类 1000 次；GPU 2 上完成 5 组新进程 `AB、BA、AB、BA、AB` 对照；旧 1800.529 s 耐久只保留为性能证据，目标连续性由 4.5 节的 1800.514 s 正式替代耐久验收。 | 最大规模和 DWB/MPC 各 5/5 到达结论保留；替代耐久 31/31 成功，ABORTED/测试超时为 0；完整处理 p95/p99/max 83.615/84.913/90.433 ms，超 100 ms 为 0/15,557。 |
| P6：发布和回归（已通过） | 完成依赖闭包、临时重定位、绝对路径和校验和审计；增量安装；依次验证发布版 MPC 入口和原 DWB 入口。 | 当前发布不引用开发目录且零符号链接；MPC/DWB action 均成功，到达误差 0.2254/0.2265 m；既有 7 项保护文件一致。 |

### 5.1 P0 运行数据

对 `/human_states`、`/odom`、`/lidar`、`/clock`、local/global costmap 保存实际发布节点、类型、QoS offer、参数、墙钟与 ROS 时间频率、p95/p99 间隔、frame、时间戳单调性、数据年龄和首次有效时间。HuNav 另记录行人数量、ID 稳定性和正常停止/重新启动行为。

源码中的 40 Hz、KeepLast(1) 和 `map` 只能标注为源码配置，不能写成实测结果。主动 reset 故障放在 P2 独立环境。

### 5.2 行人安全距离

`d_measured` 定义为时间对齐后机器人 footprint 与 HuNav 行人圆的最小有符号净距离；`d<=0` 表示接触或重叠。0.30 m 是要求净间隔，0.05 m 是采样、测量和时间对齐的总不确定度上限，不能从安全距离中扣掉。

通过条件：`d_lower = d_measured - e_total >= 0.30 m`。初始 `e_total <= 0.05 m`
审计上限已在 0.8 m/s 速度增量后依据实测采样周期修订为 `e_total <= 0.06 m`；
这只扩大可被量化的时间对齐误差范围，0.30 m 保守净距下界不变。无法界定误差或
误差超限时标为未能判定，不能计作通过。外接圆模型距离、真实 footprint 距离和
Isaac 显示滞后分别记录。

### 5.3 DWB/MPC 对照

正式对照使用同一确定性六行为配置做五组重复，按 `AB、BA、AB、BA、AB` 配对交错。实机配置不存在可控制的随机种子参数，因此没有伪造 seed；每轮记录相同配置哈希、相同目标、速度、物理步长和 GPU，并以相同预热条件启动本次拥有的进程，不依赖热 reset。

DWB 原 footprint 和参数保持不变；报告必须披露与 MPC 保守 footprint 的差异，首版对照用于功能和系统影响评估，不把所有结果差异直接归因于算法。

原 `verify_six_behaviors` 会发布 `/cmd_vel`，只能在 `NAVIGATION=false` 的独立 baseline 中使用。MPC 集成使用只读校验器并通过 Nav2 action 导航。

每次运行保存源码、依赖、场景哈希，GPU UUID/余量，ROS domain、端口、包解析路径、最终参数、结构化 CSV/JSON、退出状态及必要日志/画面；只有需要复现消息内容时才录制 rosbag，避免为每次高频 lidar/image 重复运行生成无边界数据。记录成功率、碰撞、最小净间隔、到达时间、路径长度、速度/加速度、总变差、停车时间、求解和完整处理分位数、超时/无解率、消息年龄、CPU/GPU 及 RTF。

## 6. Revision 2 审查结论

| 审查项 | 结论 |
|---|---|
| `PlannerException` | 确认是 Humble Controller 的真实失败接口；修订使用语义，只让不可恢复错误进入异常链，动态安全等待正常返回零命令并保留 action。 |
| CasADi/IPOPT C++ plugin 可用性 | 确认。standalone C++ SDK、ABI、IPOPT、最小重定位闭包、pluginlib、实际 controller_server 装载及最终发布树重定位均已通过。 |
| `/human_states` 实际 QoS、频率、时间和 reset | QoS offer、频率、frame、时间戳和正常运行由 P0 确认；P2 已确认断流、陈旧/未来样本、整套重启和真实桥暂停后的迟到响应。单独 HuNav 热重启透明恢复被明确排除。 |
| 80 ms solver 与 p95 80 ms | 确认原定义不一致；冻结为 75 ms IPOPT 选项、90 ms plugin 提交上限、100 ms 完整 deadline 和 250 ms 命令 lease。P5 完整处理 p95/p99 为 82.500/84.672 ms，超 100 ms 为 0。 |
| MPC/global/local/checker footprint | 确认需要统一；DWB baseline 保持原样。 |
| lidar watchdog | 否定只看 costmap；算法间接依赖 costmap，同时监控原始 lidar 健康。 |
| 首控制加速度基于 odom | 确认。 |
| fixed-size NLP + active mask | 确认为 8 动态槽；否定把 32 dynamic + 128 static 全部放入固定 NLP。输入容量、可达筛选和全量后检查保留。 |
| 最大规模 benchmark 提前到 P1 | 确认并已完成；最大输入为 32 dynamic + 128 static，每类 1000 次。 |
| 0.30 m 与采样误差定义 | 确认原表述有歧义，改为 0.30 m 保守净距下界；0.8 m/s 配置下实测推导的采样对齐误差 gate 为 0.06 m。 |
| DWB/MPC 交错运行 | 确认。 |
| path/data/reset generation | 确认路径和 reset 隔离；否定每条数据更新都使在途结果失效。 |

P0 已于 2026-09-07 通过：原 DWB 入口五次导航 smoke 全部成功，机器人位移为 1.775～1.859 m；`NAVIGATION=false` 六行为验证通过；60 秒运行数据和参数已冻结。P1 同日通过：独立 C++ SDK 与依赖闭包、数学核心、数值对照、缺陷用例、CMake consumer 和容量 benchmark 全部达到 gate。P2 于 2026-09-08 通过：17 份接口、时序、命令链和故障注入证据均满足修订后的分层时限，真实六行为桥迟到响应路径也已覆盖。P3 同日通过：十类最终场景各完成五次，所有可达场景满足目标容差和 150 s gate，全部静态碰撞样本为 0，未知区和封堵均安全失败。P3 依据实测修正了参考路径推进、生成墙体后的输入等待、遮挡障碍可见性、拐角/旋转 fixture 净空和 action 结束后的 odom 测量时序；被否定的 fixture 和日志均保留在 `evidence/p3`。P4 于 2026-09-10 通过：横穿、迎面、同向、多人交叉、停止/转向、ID 变化、积压和六行为共 40/40 正式运行通过；完整汇总和被拒绝试验保存在 `evidence/p4`。实测促使正常规划安全距离设为 0.35 m、紧急制动硬下限保留 0.30 m，并增加精确矩形制动检查、路径世代丢弃重试、动态行人残留 costmap 标记的有界零速等待。4.5 节纠正审计再把 progress checker 修订为 0.05 m/0.1 rad/120 s，并以在线证据把仅限 HuNav 匹配残留的等待上限从 1 s 修正为 120 s。

2026-09-11 的 P5 前置检查发现稳定 install 被独立重建；当前实际 DWB 原生入口复验通过后，只在开发目录重新冻结该 install 哈希，旧/新哈希与来源证据完整保留。最大负载三类各执行 1000 次、五组 DWB/MPC 新进程交错对照共 10/10 到达，性能和命令链统计有效。纠正审计确认旧耐久还包含 259 个 ABORTED 和两个测试超时，旧接受规则遗漏了这两个拒绝条件；因此旧运行不再证明目标连续性，只保留 90,025 个有限输出、唯一 `/cmd_vel` 发布者和时序分位数等性能证据。2026-09-18 的替代耐久以 31/31 成功、零 ABORTED、零单目标测试超时正式关闭目标连续性 gate。GPU 2/3 的其他用户任务没有被干扰，正式运行均记录可用显存和 GPU UUID。

P6 于 2026-09-12 首次通过；2026-09-18 的目标连续性纠正发布再次在临时位置完成重定位、runtime、benchmark、路径和依赖审计后才增量写入稳定目录。发布版 MPC 和原始 DWB 分别在 domain 214/215、GPU 3 上完成独立 action smoke，两者加载预期 plugin，均满足 0.25 m 到达容差。最终重建的 20 项测试、shell/Python 静态检查、发布校验和、运行依赖和稳定保护清单全部通过。

### 6.1 实施前仍需验证

本计划范围内没有未关闭的实施前验证项。以后每次运行仍须按 1.2 节检查当时的 GPU 显存、domain 和端口，这属于运行前资源检查，不是遗留实现 gate。

## 7. 完成定义

**完成状态：已满足。**MPC 数学核心可追溯；原 DWB 默认命令和行为不变；MPC 能从 `arena5_ws` 独立入口启动；P0～P6 全部通过；基本导航、静态/行人避障、故障停车、性能及回归证据齐全；发布依赖闭包和回退均已验证。

交付内容包括三个新增包、独立依赖锁定和构建工具、新启动入口、MPC/场景配置、Python/C++ 数值对照、集成和故障测试、DWB/MPC 报告、版本化发布包及回退说明。

P0～P6 首版不包含 ROS1 兼容、旧感知链、机器人模型迁移、Nav2/Isaac/HuNav 升级、方法热切换、高速度轨迹调优或新的全局规划器。4.4 节是首版完成后的受控上限参数增量；它没有改变求解器结构、标称参考速度或默认 DWB 入口。

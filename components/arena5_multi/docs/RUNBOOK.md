# 构建、启动与验收手册

## 开发工作区

```bash
cd /home/lpc/workspace/arena5_multi_ws
source scripts/env.sh
scripts/verify_baseline.py
scripts/build.sh
scripts/validate_offline.sh
```

环境入口默认使用 ROS domain `71`，WebRTC TCP/UDP `49130/48030`，Foxglove `8795`。
启动脚本会检查保护基线、端口和 NVIDIA 设备，并选择空闲显存最多的 GPU。可用
`GPU_ID`、`MULTIROBOT_MIN_FREE_MIB`、`LIVESTREAM`、`FOXGLOVE` 覆盖运行选项。

```bash
scripts/run_multirobot.sh config/scenarios/one_robot.yaml
scripts/run_multirobot.sh config/scenarios/two_robots_external.yaml
scripts/run_multirobot.sh config/scenarios/two_robots_dijkstra.yaml
scripts/run_multirobot.sh config/scenarios/four_robots.yaml
scripts/run_multirobot.sh config/scenarios/eight_robots_capacity.yaml
```

每次启动创建独立 run 目录并保存场景、Nav2 参数、源码提交、GPU、端口和 ROS 日志。
清理只向登记的独立进程组发送 `SIGINT`：

```bash
scripts/cleanup.sh
```

## 任务和控制

以下命令都在已经 `source scripts/env.sh` 的第二个终端执行：

```bash
multi_nav_goal robot_1 12.0 3.0 0.0
multi_nav_waypoints robot_1 12.0,3.0,0.0 3.0,3.0,3.14159
multi_nav_cancel robot_1
ros2 service call /robot_1/emergency_stop std_srvs/srv/SetBool '{data: true}'
ros2 topic pub -r 10 /robot_1/cmd_vel_external geometry_msgs/msg/Twist \
  '{linear: {x: 0.15}, angular: {z: 0.0}}'
```

`cmd_vel_external` 只对 external 场景生效。Nav2 场景忽略该输入；external 场景不启动
该机器人的 Nav2 节点。

场景就绪后运行 `scripts/capture_graph.sh RUN_DIR`，保存 ROS 图、action、service 和所有
节点的实际参数；benchmark 和 validator 会把 action 结果与机器可读指标写入同一目录。

## 分阶段验收

P0 单机先运行 `validate_runtime.py`；P1 使用 external 双机运行控制隔离脚本：

```bash
scripts/validate_runtime.py config/scenarios/one_robot.yaml --duration 60
scripts/validate_control.py config/scenarios/two_robots_external.yaml
scripts/validate_angular_sign.py config/scenarios/two_robots_external.yaml \
  --output RUN_DIR/angular_sign_validation.json
```

角速度符号验收要求正/负 `cmd_vel.angular.z` 分别产生正/负的解缠 yaw 和 odometry
角速度，并确认另一台机器人平移漂移不超过 `0.02 m`。

P2 在双机 Nav2 场景中执行运行图、取消隔离和连续 10 轮目标：

```bash
scripts/validate_runtime.py config/scenarios/two_robots_dijkstra.yaml --duration 60
scripts/validate_peer_obstacle.py config/scenarios/two_robots_dijkstra.yaml
scripts/validate_cancel_isolation.py config/scenarios/two_robots_dijkstra.yaml
scripts/benchmark_navigation.py config/scenarios/two_robots_dijkstra.yaml \
  config/tasks/two_robot_lanes.yaml
```

P3 对四机执行相同的运行图检查和 10 轮任务，并另行保持启动 1800 秒。八机只执行容量
压测。资源采样的 `run_dir` 是启动脚本打印的目录：

```bash
scripts/benchmark_navigation.py config/scenarios/four_robots.yaml config/tasks/four_robot_lanes.yaml
scripts/validate_runtime.py config/scenarios/four_robots.yaml --duration 1800
scripts/capture_resources.py RUN_DIR --duration 1800
scripts/validate_runtime.py config/scenarios/eight_robots_capacity.yaml --duration 600
scripts/capture_resources.py RUN_DIR --duration 600
```

P4 分别启动 `two_robots_hunav_regular.yaml` 和 `two_robots_hunav_six.yaml`。每个实例运行：

```bash
scripts/validate_hunav_runtime.py config/scenarios/two_robots_hunav_regular.yaml --duration 60
scripts/validate_hunav_runtime.py config/scenarios/two_robots_hunav_six.yaml --duration 60
```

图中只能有一套 loader、manager 和 adapter；周期状态必须报告
`reference=robot_1 interaction_scope=single_reference_robot`。随后从未加载多机 overlay 的
终端运行原 `scripts/run_six_behaviors.sh` 做旧单机回归。

P5 对同一 `two_robot_lanes.yaml` 分别启动 Dijkstra 和 A* 场景，保存两个 benchmark
JSON，再比较：

```bash
scripts/compare_algorithms.py RUN_DIJKSTRA/navigation_benchmark.json \
  RUN_ASTAR/navigation_benchmark.json --output comparison.json
```

通过条件以开发计划为准。窄道冲突、不可达目标和独立 DWB 死锁应记录失败原因，不能
解释成协同规划结果。

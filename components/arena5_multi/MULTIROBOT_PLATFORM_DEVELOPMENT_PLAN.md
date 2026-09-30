# Arena5 多机器人导航基础平台开发计划

计划日期：2026-09-23。开发目录：`/home/lpc/workspace/arena5_multi_ws`。

## 目标和边界

在同一 Isaac Sim 场景中运行多台 Jackal，每台拥有独立 ROS 2 namespace、
TF frame、规范雷达、D6 速度控制、Nav2 栈、任务 action 和安全命令链。首版采用
`0.26 m/s`、`1.0 rad/s`，正式验收规模为 4 台，8 台只做容量压测。

本版不实现协同规划、任务分配、路权、ORCA/MAPF、MPC 多机迁移、AMCL/SLAM 或
跨主机部署。HuNav 只允许一套行人状态和计算链，并固定 `robot_1` 为行人社会反应
参考机器人；其余机器人只能观测和避让同一批行人。

## 阶段

1. P0：冻结主工作区状态，建立独立 underlay/overlay 和单机基线。
2. P1：两台机器人独立生成、D6、odom、雷达、TF、轮显示和命令守护。
3. P2：两套独立 Nav2、公共地图、同伴真值障碍层和任务工具。
4. P3：4 台正式验收与 8 台容量压测。
5. P4：HuNav 有限接入，保持原有单机器人交互语义。
6. P5：NavFn Dijkstra/A* 加 DWB 的配对任务对照。
7. P6：不可变增量发布与回退检查。

## 接口契约

每台机器人使用 `/robot_i/{odom,lidar_normalized,joint_states,cmd_vel}`，Nav2 action
位于 `/robot_i/{navigate_to_pose,follow_waypoints}`。外部控制写入
`/robot_i/cmd_vel_external`，Nav2 写入内部守护输入，只有守护节点发布最终
`/robot_i/cmd_vel`。TF 使用共享 `/tf` 和 `/tf_static`，frame 固定为
`robot_i/odom -> robot_i/base_link -> robot_i/...`。共享资源只有 `/clock` 和 `/map`。

同伴障碍层使用仿真真值 odometry 和 `0.48 x 0.44 m` footprint，只表达动态占据，
不实现优先级或协同预测。同伴、规范雷达或控制输入超过墙钟期限时守护节点发布零速；
时钟回退会锁存故障并要求重启实验。

## 验收规则

- 控制隔离、唯一最终速度发布者、唯一时钟发布者和无 TF 冲突。
- 无命令机器人漂移不超过 `0.02 m`；命令断流后 `0.6 s` 内输出零速。
- action 必须为 `SUCCEEDED`，终点误差不超过 `0.25 m`、`0.25 rad`。
- 4 台每台连续 10 个目标成功并运行 30 分钟；8 台形成容量报告。
- 规范雷达按仿真时间为 `9.5--10.5 Hz`，同时记录 RTF、墙钟频率和资源使用。
- 相向窄道和不可达任务允许安全停车或失败，不将其描述为协同能力。

GPU 验收已于 2026-09-24 完成：P0–P5 通过，4 台为正式支持规模；8 台完成 600 秒容量压测并确认达到当前容量边界。P6 发布和恢复结果单独记录在 `evidence/RELEASE_VALIDATION.md`。


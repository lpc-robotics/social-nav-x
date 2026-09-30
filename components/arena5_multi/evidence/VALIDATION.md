# GPU 验收总表

验收日期为 2026-09-23 至 2026-09-24，主机 GPU 选择为 `GPU_ID=3`。正式支持规模为
1、2、4 台；8 台结果按用户授权作为容量边界，不作为可导航规模承诺。

| 阶段 | 结果 | 主要证据 |
|---|---|---|
| P0 隔离单机 | 通过 | underlay 保护 30 项；单机 60 s RTF `0.5769`，雷达仿真频率 `10 Hz`，漂移 `0 m`，TF/Nav2 就绪 |
| P1 双机控制 | 通过 | 控制隔离、异速并行、单机停止、急停均通过；命令断流 `0.527 s` 内输出零速，物理停车 `0.682 s` |
| P2 双机独立 Nav2 | 通过 | 10/10 轮、20/20 action 成功；取消一机时另一机成功；同伴旧占据清除、自身排除均通过 |
| P3 四机正式验收 | 通过 | 10/10 轮、40/40 action 成功；30 分钟健康检查通过；最终源码另有 60 s 干净健康复验 |
| P3 八机容量压测 | 容量边界 | 600 s 完成，RTF `0.1201`；8 台 odom/TF/雷达存在，但 7 套 Nav2 未激活且守护检测到雷达陈旧，不能作为支持配置 |
| P4 HuNav 有限接入 | 通过 | 单行人与六行为两场景均通过 60 s；唯一 loader/manager/adapter，固定 `robot_1` 参考；旧单机六行为回归通过 |
| P5 算法对照 | 通过 | Dijkstra 与 A* 各 10/10 轮、20/20 action 成功，使用相同任务哈希和 DWB |
| P6 发布与恢复 | 通过 | `20260924-bdd959da` 的 319 个文件哈希通过；隐藏开发目录 GPU smoke 与原 DWB action smoke 均通过 |
| 离线与构建 | 通过 | 5 包选择性构建；配置 55 项、HuNav 8 项、pytest 10 项全部通过 |

## 关键运行证据

- P0：`logs/runs/20260923_172101_one_robot_gpu3/runtime_validation.json`
- P1：`logs/runs/20260923_174708_two_robots_external_gpu3/control_validation.json`
- 角速度符号：`logs/runs/20260924_170025_two_robots_external_gpu3/angular_sign_validation.json`
- P2：`logs/runs/20260924_101114_two_robots_dijkstra_gpu3/`
- 四机 30 分钟：`logs/runs/20260924_104230_four_robots_gpu3/runtime_validation_30min.json`
- 四机最终导航：`logs/runs/20260924_170641_four_robots_gpu3/navigation_benchmark.json`
- 四机最终源码健康复验：`logs/runs/20260924_172321_four_robots_gpu3/runtime_validation_60s.json`
- 八机容量：`logs/runs/20260924_172732_eight_robots_capacity_gpu3/`
- HuNav 单行人：`logs/runs/20260924_213139_two_robots_hunav_regular_gpu3/`
- HuNav 六行为：`logs/runs/20260924_214037_two_robots_hunav_six_gpu3/`
- P5 Dijkstra：`logs/runs/20260924_214853_two_robots_dijkstra_gpu3/navigation_benchmark.json`
- P5 A*：`logs/runs/20260924_215532_two_robots_astar_gpu3/navigation_benchmark.json`
- 算法汇总：`evidence/algorithm_comparison_20260924.json`

四机 30 分钟运行基于提交 `2663fd3`，最终导航基于 `7d3c534`；其后只修改了资源采样中断保存和
HuNav 验收器清理逻辑。最终控制与导航源码已通过独立 60 秒四机健康复验。P4 的旧单机回归从
未加载多机器人 overlay 的主工作区运行，输出为
`SIX_BEHAVIORS_VERIFY_OK ... responses=3,4,5,6 robot_distance=0.990`。

八机报告中的 `passed: false` 是容量结论的一部分，不能改写为正式通过。详细限制见
`docs/EIGHT_ROBOT_CAPACITY_REPORT.md`。

## 2026-09-28 感知验收修正

在线发现旧多机版本漏配 `voxel_layer.normalized.max_obstacle_height`，Humble 实际默认为 0.0，过滤约 0.349 m 高的规范雷达命中。旧 P4 集成通过不证明行人已进入代价地图，旧导航成功也不代表雷达避障有效。修复提交 `e9d118c` 显式设置观测源高度 0.0–2.0 m；配置增量发布 `20260928-e9d118c-heightfix`，双机实测证据见 `pedestrian_costmap_heightfix_20260928.json`。需对修复后的完整动态避障和清除另行验收，不能沿用旧 P2/P4 的结果作为本次全量验收。

# Release 20260924-bdd959da 验证记录

- 发布路径：`/home/lpc/workspace/arena5_ws/optional/multirobot/releases/20260924-bdd959da`
- 发布源码提交：`bdd959dad84b7d4a810f25fe2a711d78eb9c3e53`
- `SHA256SUMS`：319 个文件全部通过。
- 稳定启动入口：`/home/lpc/workspace/arena5_ws/scripts/run_multirobot.sh`，SHA-256
  `1d65a11817871ddff6cf98a330fa4ace4848bb664dfc1bb4978f7f43f11895aa`。
- 稳定清理入口：`/home/lpc/workspace/arena5_ws/scripts/cleanup_multirobot.sh`，SHA-256
  `0331deb1afaee0b0e76acead6c8cdb8779a63948b3ff31b172b3f6f194eed6d3`。

## 隐藏开发目录 GPU 演练

`/home/lpc/workspace/arena5_multi_ws` 临时改名后，从稳定入口启动 release 的
`one_robot.yaml`。运行目录为：

`/home/lpc/workspace/arena5_ws/.multirobot/logs/runs/20260924_221436_one_robot_gpu3`

60 秒验收通过：唯一 `/clock` 与 `/robot_1/cmd_vel` 发布者，RTF `0.5984`，雷达仿真频率
`10 Hz`、墙钟频率 `5.9804 Hz`，命令守护约 `20 Hz` 且无观测故障，出生误差与空闲漂移均
为 `0 m`，TF 和 NavigateToPose action 就绪。报告副本为
`evidence/release_runtime_validation_20260924.json`。实例停止后开发目录已恢复。

首次候选包 `20260924-32cf2fe8` 在同样演练中暴露单机场景空 `peer_names` 参数未初始化，
`command_guard` 因此退出。问题在提交 `bdd959d` 修复；该候选包保留作审计，稳定入口不引用它。

## 原入口恢复演练

在未加载多机器人 overlay 的终端启动原
`/home/lpc/workspace/arena5_ws/scripts/run_six_behaviors.sh`，运行目录为：

`/home/lpc/workspace/arena5_ws/logs/runs/20260924_221727_six_behaviors_gpu3`

实际参数确认 `FollowPath.plugin=dwb_core::DWBLocalPlanner`，NavFn 的
`GridBased.use_astar=true`。向原 `/navigate_to_pose` 发送 `(4.0, 3.0, 0.0)` 后 action 返回
`SUCCEEDED`。此前 P4 旧单机回归还通过了 `verify_six_behaviors`，观测到 3/4/5/6 特殊响应。

发布后再次执行 underlay 保护检查，30 项全部通过。原 DWB、DWB 0.8、MPC 入口、关键安装
文件和 11 个嵌套仓库状态与 P0 记录一致；旧多机 release `20260923-2961640` 也未覆盖。

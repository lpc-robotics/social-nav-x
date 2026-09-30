# Windows 人工可视化固定入口

人工交付默认：Foxglove `ws://localhost:8765`；WebRTC Server
`10.16.205.165`，Signal `49100`，Stream `47998`，客户端选择 `1920 x 1080 (FHD)`。

Linux 新终端启动双机：

```bash
cd /home/lpc/workspace/arena5_ws
GPU_ID=3 ./scripts/run_multirobot.sh
```

Windows PowerShell 每次建立连接并保持窗口运行：

```powershell
ssh -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=30 -o ServerAliveCountMax=3 -L 8765:127.0.0.1:8765 lpc@10.16.205.165
```

也可在 Windows 执行随交付提供的 `connect_multirobot.ps1`。
Foxglove 选择 Foxglove WebSocket，保存地址 `ws://localhost:8765`。
SSH 将 Windows 本地 8765 转发到服务器本地 8765；服务器启动命令无法自动建立 Windows 的隧道。
如果已有指向正确服务器的相同隧道，直接复用。端口被其他程序占用时脚本明确失败，不自动换端口。
单机和多机人工实例共享这些端口，需先退出旧实例再启动另一种实例。

3D 面板使用 `map`，Pose 发布话题分别为 `/robot_1/goal_pose` 与
`/robot_2/goal_pose`，类型 `geometry_msgs/msg/PoseStamped`。

程序测试继续使用开发目录的隔离端口，也可显式覆盖
`ARENA_WEBRTC_SIGNAL_PORT`、`ARENA_WEBRTC_MEDIA_PORT`、`ARENA_FOXGLOVE_PORT`。
人工入口尊重显式环境覆盖；请使用未加载开发环境的新终端，避免继承旧端口。
`./scripts/run_multirobot.sh --check-visualization` 可查看入口配置而不启动仿真。

本次仅更新可变交付入口，不修改 20260924-bdd959da 不可变发布包。
旧入口副本在 `evidence/visualization_20260928/run_multirobot.before.sh`。

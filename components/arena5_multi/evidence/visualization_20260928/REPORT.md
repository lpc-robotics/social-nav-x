# 人工可视化入口验证 2026-09-28

- 修改范围：主工作区可变 scripts/run_multirobot.sh，以及 Windows connect_multirobot.ps1；对应源码保存在独立开发仓库 scripts/delivery。
- 不可变发布包 20260924-bdd959da 未修改；旧入口已备份。
- bash -n 通过；--check-visualization 默认输出 localhost:8765、10.16.205.165、49100、47998。
- stable underlay 保护检查：30/30 通过。
- GPU 3 实际启动双机；Foxglove 127.0.0.1:8765、Isaac TCP 0.0.0.0:49100 已监听。
- Foxglove SDK WebSocket 握手返回 101，证据 handshake.txt。
- Isaac 实际参数包含 --webrtc-signal-port 49100 --webrtc-media-port 47998，Kit 参数 fixedHostPort=47998；输出 app ready。
- 尚未完成 Windows 客户端端到端画面验证；检查时 UDP 47998 尚未出现在监听列表，需客户端连接后确认媒体流。
- 握手探针读完响应后关闭，导致 Bridge 的 Broken pipe/未发送关闭帧日志，属于本次探针行为。
- 实测实例保留运行供用户连接，运行记录：arena5_ws/.multirobot/logs/runs/20260928_161043_two_robots_dijkstra_gpu3。

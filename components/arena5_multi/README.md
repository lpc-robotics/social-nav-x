# Arena5 multi-robot navigation platform

普通行人的多机器人社会力后端与心理模型扩展接口见
[MULTI_SFM.md](docs/MULTI_SFM.md)。新模式单独配置、单独验收；原 HuNav 六行为模式保留。

This repository is the isolated development and release source for the Arena5
multi-robot platform. The deployed workspace at
`/home/lpc/workspace/arena5_ws` is a read-only underlay during development.

The first supported topology is one Isaac Sim 5.1 process, one ROS clock, a
shared map, and independently namespaced Jackal/Nav2 stacks. Configuration is
data-driven. Existing legacy-platform evidence covers 1, 2, and 4 robots; the
8-robot scenario is a capacity probe. The new `multi_sfm` backend has its own
acceptance in `evidence/multi_sfm`, scoped to two robots and 1/6 ordinary people
at 0.26 m/s and 1.0 rad/s. Earlier platform evidence does not certify this backend.

```bash
cd /home/lpc/workspace/arena5_multi_ws
source scripts/env.sh
scripts/verify_baseline.py
scripts/build.sh
scripts/validate_offline.sh
scripts/run_multirobot.sh config/scenarios/two_robots_dijkstra.yaml
```

GPU-backed validation status and exact acceptance gates are recorded in
`MULTIROBOT_PLATFORM_DEVELOPMENT_PLAN.md` and `evidence/VALIDATION.md`. Runtime
interfaces and phase commands are in `docs/INTERFACES.md` and `docs/RUNBOOK.md`.

人工可视化统一入口与 Windows 连接方法见 [WINDOWS_VISUALIZATION.md](docs/WINDOWS_VISUALIZATION.md)。
正式入口默认 Foxglove localhost:8765、WebRTC 49100/47998；开发测试保留隔离端口。

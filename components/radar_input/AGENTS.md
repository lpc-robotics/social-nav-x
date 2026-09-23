# 雷达与代价地图任务交接

处理本仓库的 LaserScan、`/lidar_clearing`、Nav2 代价地图或相关启动脚本前，先读 [RADAR_INPUT_HANDOFF.md](RADAR_INPUT_HANDOFF.md)。该文档区分已验收的规范雷达方案与冻结的 `depth_clearing` 稳定基线，并给出源码位置、运行入口、验证边界及回退方式。

不要将 `.workspaces/laserscan-v1` 中的隔离实验改动直接写进稳定的 `src/` 工作树；不要移动或覆盖已有的 `baseline/`、`validated/`、`accepted/` 标签。保留用户未提交文件和 `src/arena/simulation-setup` 现有工作树差异。项目扩展代码和隔离 overlay 属于工作区，NVIDIA Isaac Sim 安装目录及 Conda 包不是本方案的修改目标。

# 发布与恢复

开发目录是 Git 仓库，稳定工作区只作为 underlay。`baseline/stable_underlay.json` 保存
11 个嵌套仓库的提交、分支和完整工作树状态，以及原 DWB、DWB 0.8、MPC 入口和关键
安装文件哈希。任何启动和发布前都运行 `scripts/verify_baseline.py`；失败时停止并重新
核查，不能覆盖用户修改或更新基线来绕过错误。

发布脚本只接受干净提交，将源码快照、配置、文档、工具和一个按最终绝对路径构建的
overlay 写入：

```bash
scripts/create_release.sh
```

最终入口固定指向具体 release ID，不使用可移动的 `latest`。发布目录写保护并包含
`RELEASE.json` 和 `SHA256SUMS`。其 overlay 直接构建到最终路径，因此把
`/home/lpc/workspace/arena5_multi_ws` 临时改名后仍能运行。

恢复原平台时先运行多机 release 的 `scripts/cleanup.sh`，打开一个未 source 多机 overlay
的新终端，执行稳定工作区原入口。无需卸载或重建 underlay：

```bash
cd /home/lpc/workspace/arena5_ws
source scripts/env.sh
scripts/run_six_behaviors.sh
```

移除主入口只影响多机增量；已有三个单机入口和既有发布包不在多机发布过程中修改。

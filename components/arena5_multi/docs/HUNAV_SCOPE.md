# HuNav 有限接入边界

多机器人平台只启动一套 `hunav_loader`、`hunav_agent_manager` 和
`arena_multi_hunav` 适配器。场景管理器先生成墙体与全部机器人，适配器随后只生成、
计算和显示一批行人。适配器显式移除了旧单机桥的全局 `/odom`、`/cmd_vel` 订阅和轮速
发布职责，轮显示由逐机 `wheel_visualizer` 完成。

HuNav Humble 的 `ComputeAgents` 请求只有一个 `robot` 字段。本版固定把
`/robot_1/odom` 转成该字段；行人只会对 `robot_1` 产生原有社会反应。`robot_2...N`
通过各自规范雷达观察并避让相同行人，但行人不会针对它们获得完整的社会反应。
regular 和 six 配置用于验证单套状态的一致性，不能据此宣称完整多人—多机器人交互。

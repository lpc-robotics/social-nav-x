# 速度上限交付检查

发布版：20260928-9dbbb36-speedlimits；源码提交 9dbbb36。

- 命令守护：线速度绝对值 1.0 m/s，角速度绝对值 1.2 rad/s。
- D6 底盘默认限幅同步；显式 ARENA_IDEAL_MAX_LINEAR/ANGULAR 环境覆盖仍有效。
- DWB max_vel_x/max_speed_xy=1.0，max_vel_theta=1.2。
- 平滑器 max_velocity=[1.0,0,1.2]，min_velocity=[0,0,-1.2]；保留导航不倒车策略。
- 恢复行为 max_rotational_vel=1.2；加减速参数未改。
- 实际新安装包的 CommandGuard._accept 在 nav2/external 模式下，正负超限截断、限内透传和 NaN 拒绝均通过。
- 新安装包导航参数一致性、D6 默认值、Python 语法及 heightfix 保留检查通过。
- 本轮未启动 Isaac，未进行 1.0 m/s 高速导航与物理速度验收。既有低速验收不能代替高速验收。
- 主工作区 command.txt 四处多机发布路径以及正式启动入口统一更新。
- command.before.txt 保留原命令文件；不可变旧发布包未修改。

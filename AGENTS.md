# RM65B 迁移仓库工程约束

- 目标环境固定为 Ubuntu 22.04、ROS 2 Humble、Gazebo Harmonic。
- `src/` 只放项目自建适配包；瑞尔曼、`gz_ros2_control` 等上游源码通过 `upstream/*.repos` 导入。
- 不把 `build/`、`install/`、`log/`、教学算法和官方测试资产提交到仓库。
- `src/rm_description` 必须是指向 `upstream/realman_ros2/rm_description` 的符号链接。
- Humble + Harmonic 需要从源码构建锁定的 `gz_ros2_control`；不要安装 Fortress 的控制插件后直接运行。
- Gazebo 反馈是执行状态来源；RViz 不能替代仿真关节反馈。
- 当前仓库只验证仿真，不包含 RS485/Modbus 驱动，也不授权连接 RM65B 实机。
- 修改后至少运行：

  ```bash
  python3 scripts/verify_migration_manifest.py
  bash scripts/verify_environment.sh
  ```

  目标系统完成构建后，再运行两个 `verify_realman_*` 运行时检查。

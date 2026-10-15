# 上游来源目录

这里不保存上游源码。执行 `scripts/import_upstream.sh` 后，厂商仓库会按
`realman_humble.repos` 中的完整提交导入到 `realman_ros2/`；该目录由
`COLCON_IGNORE` 排除，不直接作为项目自建包编译。

RM65B 模型入口通过 `src/realman_rm65/rm_description` 符号链接指向该导入目录。上游仓库保持原样，
项目的 SRDF、控制器、Gazebo 和启动适配放在 `src/` 的项目包中。

知行夹爪原始仓库通过 `https://gitee.com/Ice_Panda/changingtek.git` 导入到
`upstream/changingtek/`，当前快照为 `dd27be09df77be843c15a1133b0e11130b31a7b2`。
该仓库是 ROS 1 Catkin 资产库，已由 `COLCON_IGNORE` 排除；ROS 2 Humble 使用
`src/crt_ctm2f110_gripper_visualization/robot_changingtek_description` 中的无运行时驱动适配包，避免把 ROS 1 节点和
Gazebo Classic 依赖带入 Harmonic 工作空间。

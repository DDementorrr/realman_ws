# 上游来源目录

这里不保存上游源码。执行 `scripts/import_upstream.sh` 后，厂商仓库会按
`realman_humble.repos` 中的完整提交导入到 `realman_ros2/`；该目录由
`COLCON_IGNORE` 排除，不直接作为项目自建包编译。

RM65B 模型入口通过 `src/rm_description` 符号链接指向该导入目录。上游仓库保持原样，
项目的 SRDF、控制器、Gazebo 和启动适配放在 `src/` 的项目包中。

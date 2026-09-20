# Humble + Harmonic 依赖锁定说明

ROS 2 Humble 的默认 Gazebo 是 Fortress。Harmonic 组合需要安装 OSRF 的
`gz-harmonic`、`ros-humble-ros-gzharmonic`、`ros-humble-ros-gzharmonic-bridge`
和 `ros-humble-ros-gzharmonic-sim`，并从 `gz_ros2_control` 的 `humble` 分支源码构建。

本迁移提交不把 `gz_ros2_control` 源码复制进仓库。源码提交锁定为
`522ca0e0d3560dc8c52628e9f52b86db78673920`，该提交来自官方 Humble CI 页面；脚本默认使用
该哈希，也允许通过 `GZ_ROS2_CONTROL_COMMIT` 显式替换。脚本会把最终来源写入
`.third_party/gz_ros2_control.commit`。

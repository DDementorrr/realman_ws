# RM65B + CTM2F110 ROS 2 Humble 迁移工作空间

这个仓库保存从主工作空间迁移到 Ubuntu 22.04 / ROS 2 Humble 的最小工程集合：

- 瑞尔曼 RM65B 的项目 MoveIt 2 配置；
- 从 ROS 1 资产适配而来的 CTM2F110 夹爪描述包；
- Gazebo Harmonic 控制模型、联动关节 world 和启动编排；
- MoveIt 目标位姿适配器、交互目标发布器以及运行时检查脚本。

厂商 RM65B 描述、ROS 2 接口和驱动不复制到本仓库，而是由
[`upstream/realman_humble.repos`](upstream/realman_humble.repos) 固定来源和提交后再导入。
MoveIt、OMPL、RViz 和 Gazebo 采用 Ubuntu/ROS 官方二进制包。这样仓库只承担项目自建
适配层，不伪装成瑞尔曼官方驱动，也不包含可直接控制实机的硬件接口。

## 目标环境

目标组合是 Ubuntu 22.04、ROS 2 Humble、Gazebo Harmonic、MoveIt 2 和 `ros2_control`。
Humble + Harmonic 是一个需要显式安装 Gazebo 非默认二进制包、并从源码构建
`gz_ros2_control` 的组合；不要把 Fortress 的 `ros-humble-gz-ros2-control` 二进制
误当成 Harmonic 插件。

## 快速复现

```bash
cd ~/realman_ws
bash scripts/setup_ubuntu2204_ros2_humble.sh
bash scripts/import_upstream.sh
bash scripts/prepare_realman_model_source.sh
# 可选：用另一个已验证的完整提交替换默认锁定值
# export GZ_ROS2_CONTROL_COMMIT=<经过验证的40位提交>
bash scripts/build_gz_ros2_control_harmonic.sh
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys="ros_gz ros_gz_bridge ros_gz_sim gz_ros2_control"
colcon build --symlink-install
source install/setup.bash
ros2 launch manipulation_bringup realman_moveit_gz.launch.py \
  with_gripper:=true
```

没有图形服务器时使用 `headless:=true use_rviz:=false`。仿真运行后，另开终端执行：

```bash
source /opt/ros/humble/setup.bash
source ~/realman_ws/install/setup.bash
bash scripts/verify_realman_gz_runtime.sh
bash scripts/verify_realman_gripper_gz_runtime.sh
```

## 迁移范围

具体判定见 [迁移清单](docs/迁移记录/迁移清单.md) 和
[迁移报告](docs/迁移记录/2026-09-20-ROS2_Humble_RM65B夹爪迁移报告.md)。
其中明确区分项目自建内容、只记录来源的官方内容、教学算法和不应迁移的生成目录。

本仓库仅做 Gazebo 和 MoveIt 仿真。没有连接实机、没有启用串口/Modbus，并且
CTM2F110E 与仓库中 CTM2F110 模型的等同性、法兰安装变换和真实负载参数尚未确认。

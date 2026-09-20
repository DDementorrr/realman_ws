# 瑞尔曼 RM65B 的 RViz 与 Gazebo 联合仿真

## 1. 本阶段完成了什么

本阶段只接入 RM65B 标准版裸臂：RViz 负责交互目标和规划显示，MoveIt/OMPL 负责规划，
`rm65b_arm_controller` 通过 `gz_ros2_control` 驱动 Gazebo 中的六个关节，Gazebo 的
`/joint_states` 再同步回 MoveIt 和 RViz。

厂家源码固定在 `manipulation_ws/upstream/realman_ros2`，当前提交见 `upstream.repos`。
仓库说明面向 Ubuntu 22.04 与 ROS 2 Humble，因此没有直接启动厂家整仓；当前 Jazzy +
Gazebo Harmonic 的 SRDF、控制器和启动编排由项目包维护。厂家仓库中的 `rm_description`
保持原样，通过 `src/rm_description` 符号链接进入构建，只提供官方网格和 URDF 快照。
全新工作空间导入厂家源码后，运行 `/home/ww/scripts/prepare_realman_model_source.sh` 创建该入口；
脚本遇到已有非预期路径时会拒绝覆盖。

本页描述默认的裸臂路径。现在可显式启用基于社区 CTM2F110 ROS1 模型的研究用夹爪仿真，
方法见[RM65B 与知行夹爪联合仿真](瑞尔曼RM65B与知行夹爪联合仿真.md)。系统 PDF
记录的实体型号为 CTM2F110E，两者是否完全一致尚未证实；实机夹爪仍未接入。

## 2. 启动

```bash
cd /home/ww/manipulation_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to manipulation_bringup robot_realman_moveit_config
source install/setup.bash
ros2 launch manipulation_bringup realman_moveit_gz.launch.py
```

默认使用 `RRTConnectkConfigDefault`，速度和加速度缩放均为 0.15。Gazebo 初始保持暂停；
机械臂生成成功后控制器先配置为 inactive，世界解除暂停后再激活控制器，最后才启动
MoveIt、RViz 和目标执行适配器。某个门控步骤失败会终止整个启动。

无图形回归命令：

```bash
ros2 launch manipulation_bringup realman_moveit_gz.launch.py use_rviz:=false headless:=true
```

启动保持运行时，在另一终端执行：

```bash
/home/ww/scripts/verify_realman_gz_runtime.sh
```

## 3. RViz 操作

在 MotionPlanning 面板选择 `rm65_arm`。使用交互标记拖动 Link6 目标，先执行 Plan，检查
轨迹和碰撞结果，再执行 Execute。RViz 显示的当前姿态来自 Gazebo 反馈，不是 FakeSystem。

全零姿态同时落入厂家文档描述的腕部奇异条件。默认仿真初始关节值为
`[0.0, 0.5, -1.0, 0.0, 0.5, 0.0]`，它是项目仿真值，不是厂家标定点。

## 4. 参数化和实时输入

启动文件同时监听 `/manipulation/rm65b/target_pose`。可以使用现有交互工具，以米和弧度输入
`x y z roll pitch yaw`：

```bash
source /opt/ros/jazzy/setup.bash
source /home/ww/manipulation_ws/install/setup.bash
ros2 run manipulation_demo interactive_target --ros-args \
  -p topic:=/manipulation/rm65b/target_pose -p frame_id:=world
```

本次回归使用了接近初始末端姿态的小位移目标：

```bash
ros2 topic pub --once /manipulation/rm65b/target_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: world}, pose: {position: {x: -0.07, y: 0.0, z: 0.78}, orientation: {x: 0.0, y: 0.0, z: 1.0, w: 0.0}}}"
```

该点仅用于当前空世界仿真回归，不能照搬到实机。

## 5. 模块对应关系

| 职责 | 位置 |
|---|---|
| 厂家模型、网格与惯量快照 | `upstream/realman_ros2/rm_description` |
| RM65B SRDF、IK、限位和 MoveIt controller 映射 | `src/robot_realman_moveit_config` |
| Harmonic ros2_control 与仿真 controller | `src/robot_gazebo/urdf/rm65b_gz.urdf.xacro`、`config/rm65b_controllers.yaml` |
| 生命周期和组件组合 | `src/manipulation_bringup/launch/realman_moveit_gz.launch.py` |
| 参数目标转 MoveGroup 请求 | `src/manipulation_moveit_adapter` |
| 终端交互输入 | `src/manipulation_demo` |

## 6. 实机接入前的停止线

当前没有启用 `rm_driver`、没有机械臂 IP 参数，也不会向实体控制箱发送数据。实机阶段必须
单独核对控制器固件与厂家 ROS2 版本、急停、上电/使能状态、网络隔离、工具和负载、安装
方向、软限位、碰撞几何、速度限制以及轨迹取消行为。首次运动应在厂家允许的低速模式、
清空工作区并有人监护的条件下完成；仿真结果不能替代这些检查。

由于实体机器人主控和厂家 ROS2 合同均为 Humble，后续优先把厂家驱动放在独立 Humble
工作空间，并通过经过版本核对的 ROS 接口接入当前规划层；不得假定 Humble 自定义消息与
Jazzy 天然兼容，也不得为了省事直接把整个厂家驱动迁入当前默认构建。

参考：
[厂家 ROS2 快速开始](https://develop.realman-robotics.com/robot/ros2/getStarted/)、
[RM65 参数与 MDH 模型](https://develop.realman-robotics.com/robot/robotParameter/RM65OntologyParameters/)、
[厂家 ROS2 仓库](https://github.com/RealManRobot/ros2_rm_robot)。

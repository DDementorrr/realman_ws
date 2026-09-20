# MoveIt 资源、源码与工程构建说明

## 可以直接使用吗？

可以。`moveit_resources` 是 MoveIt 官方维护的测试/示例资源集合，包含 Panda、Fanuc M-10iA、PR2 的 URDF/XACRO、网格和 MoveIt 配置。当前 Jazzy 工程使用它的 `ros2` 分支，并已在 `manipulation_ws` 中成功构建。

它不是实际机械臂厂商驱动，也不是 MoveIt 主工程。Panda 资源 README 也提示：该配置主要用于 MoveIt 测试；真实 Panda 项目应优先使用 Franka 官方的机器人驱动和配置。

## 已导入资源

| 包 | 用途 |
|---|---|
| `moveit_resources_panda_description` | Panda URDF/XACRO、视觉/碰撞网格 |
| `moveit_resources_panda_moveit_config` | Panda 的 SRDF、IK、OMPL、控制器、RViz 和 launch |
| `dual_arm_panda_moveit_config` | 双 Panda 研究/测试配置，不代表完整双臂硬件系统 |
| Fanuc/PR2 对应包 | 其他经典模型的描述和测试配置 |

复现导入：

```bash
cd /home/ww/manipulation_ws/src
vcs import < ../moveit_resources.repos
```

构建与加载：

```bash
cd /home/ww/manipulation_ws
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch moveit_resources_panda_moveit_config demo.launch.py
```

## 环境中的 MoveIt 与主工程源码

APT 安装的 `ros-jazzy-moveit` 是已经编译好的 Jazzy 二进制发行包，提供 C++ 库、ROS 节点、MoveGroup、OMPL/控制器插件、消息和 Setup Assistant。它安装在 `/opt/ros/jazzy`，用于直接开发和运行。

`https://github.com/moveit/moveit2` 是 MoveIt 2 的主源码仓库，包含这些库和节点的源代码。只有在需要修改 MoveIt 内部实现、调试尚未发布的功能、提交补丁或从源码统一编译依赖时，才需要将主仓库放进独立源码工作空间。当前 Level 2 不需要替换已验证的 APT 二进制环境。

不要执行教程旧页面中的 `apt remove ros-$ROS_DISTRO-moveit*`，除非明确要建立源码开发工作空间；移除二进制会增加依赖和排错范围。

## Tutorials 的作用

`moveit2_tutorials` 是教学代码和分步说明，不是运行时依赖，也不是机器人模型库。它适合：

- 学习 MoveGroup、PlanningScene、RobotModel、运动学和轨迹执行 API；
- 复制一个最小示例到 `manipulation_core`/`demo`；
- 对照验证 launch、参数和接口调用。

教程 `main` 可能跟随最新 MoveIt 开发状态；Jazzy 工程应优先选择与 Jazzy 文档/依赖匹配的版本，并在独立教程工作空间构建。不要把整套教程及其源码依赖直接混入产品工作空间。

## 机器人配置文件组成

一个可运行的 MoveIt 机器人通常至少需要：

1. `URDF`/`XACRO`：连杆、关节、惯性、视觉/碰撞几何、关节限位和坐标系。
2. `SRDF`：planning groups、虚拟关节、末端执行器、禁碰对和默认状态。
3. `kinematics.yaml`：规划组的 IK 插件与参数。
4. `joint_limits.yaml`：速度/加速度等规划限位覆盖。
5. `ompl_planning.yaml`：规划器、采样和规划时间参数。
6. 控制器配置：`moveit_controllers.yaml`、`ros2_controllers.yaml` 及硬件/仿真接口。
7. launch 和 RViz 配置：robot_description、robot_state_publisher、move_group、RViz、控制器启动顺序。
8. 可选的 Cartesian limits、感知传感器、Gazebo/ros2_control 插件配置。

## 推荐模型选择

- Franka Panda：MoveIt 教程最成熟，7 自由度，适合 API/规划学习；但不是六轴。
- Fanuc M-10iA：官方资源中有，工业六轴，适合当前六轴 Level 2 验证。
- Universal Robots UR5/UR5e：厂商/社区 ROS 2 描述和 MoveIt 配置成熟，接近常见真实项目；使用前固定具体仓库与 Jazzy 分支。
- KUKA、xArm：适合后续按真实设备选型，不建议当前同时导入多个驱动栈。

当前建议先用 Panda 跑通 MoveIt API，再用 Fanuc M-10iA 或 UR5 类六轴模型完成项目验收；模型切换应只替换 adapter/config，不改变 manipulation core。

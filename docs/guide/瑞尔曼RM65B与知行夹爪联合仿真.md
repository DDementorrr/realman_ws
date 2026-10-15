# RM65B 与知行 CTM2F110 夹爪的 ROS 2 联合仿真

## 范围先说清楚

这里完成的是 **RM65B + CTM2F110 模型的 RViz、MoveIt 2、Gazebo Harmonic 仿真路径**，
不是 CTM2F110E 实机驱动。系统说明 PDF 把实体夹爪写为 `CTM2F110E`；本次取得的 Gitee
ROS1 仓库只有 `CTM2F110` 资源，尚未证明两者的外形、安装孔、质量和通信合同完全相同。
夹爪到 RM65B `Link6` 的固定关节目前采用 `xyz=0 0 0, rpy=0 0 0` 的项目仿真假设，
不是实测法兰坐标。实机未连接、未使能、未收到命令。

该 [Gitee 仓库](https://gitee.com/Ice_Panda/changingtek)不是可确认的厂家主仓。它作为社区 ROS1 资源快照导入
`realman_ws/upstream/changingtek_ros1`；本环境的 Git 克隆通道超时，实际导入使用
Gitee `master.zip`，ZIP SHA-256 和复现限制见
`realman_ws/upstream/changingtek_ros1.归档锁定.md`。原目录被 `COLCON_IGNORE` 隔离，
不要在 ROS 2 Humble 中直接构建 ROS1 资源或执行其中的串口脚本。

## 文件与数据来源

| 内容 | 项目文件 | 来源等级 |
|---|---|---|
| 外形网格、关节几何、质量与惯量快照 | `src/crt_ctm2f110_gripper_visualization/robot_changingtek_description/meshes` 和 `urdf/upstream` | Gitee ROS1 资源；仅将资源 URI 改为 ROS2 包名 |
| RM65B 模型 | `src/realman_rm65/rm_description` | 瑞尔曼厂家 Humble 仓库的未修改描述包 |
| `Link6` 到夹爪基座位姿 | `src/realman_rm65/robot_realman_moveit_config/urdf/rm65b_ctm2f110.urdf.xacro` | 项目仿真占位值；待测量 |
| 手臂和夹爪的规划组、碰撞对、执行映射 | `src/realman_rm65/robot_realman_moveit_config/config/rm65b_ctm2f110*` | 项目 ROS2 适配 |
| Gazebo 控制器、初始关节值与专用 world | `src/realman_rm65/robot_gazebo` | 项目仿真配置 |
| ROS 2 实机接口 | `src/crt_ctm2f110_gripper_visualization/changingtek_gripper_driver` | 官方 SDK 快照 + action/state 包装；默认 dry-run |

原资源的夹爪有一个主动关节 `ctm2f110_finger_joint`，另有五个联动关节：
`rightout`、左右 `kckle` 与主动关节同向；左右 `inn` 与它反向。
这里没有把六个关节当成六个可独立控制的电机。ROS1 提供的惯量仅是该资源快照中的
数值，不能拿来宣称 CTM2F110E 实机的厂家标定动力学。

## 启动与验证

裸臂默认路径保持原样；显式加 `with_gripper:=true` 才装配夹爪：

```bash
cd /home/ww/realman_ws
source /opt/ros/humble/setup.bash
source .third_party/gz_ros2_control/install/setup.bash
colcon build --packages-select robot_changingtek_description robot_realman_moveit_config robot_gazebo manipulation_bringup changingtek_gripper_driver --symlink-install
source install/setup.bash
ros2 launch manipulation_bringup realman_moveit_gz.launch.py with_gripper:=true
```

无图形服务器时可加 `headless:=true use_rviz:=false`。保持启动运行，在第二终端执行：

```bash
bash scripts/verify_realman_gz_runtime.sh
python3 scripts/verify_realman_gripper_gz_runtime.py
```

第二个脚本只读取控制器、action 名称和 `/joint_states`，核对五个关节的联动误差，
**不会发送运动指令**。两份检查均通过，才说明当前启动的状态和接口就绪；它们不能代替
真实轨迹执行实验。

## MoveIt 和 RViz 中怎样操作

RViz 的 MotionPlanning 面板中：

- `rm65_arm` 是六轴臂，仍可拖动 `Link6` 的交互目标，Plan 后再 Execute。
- `rm65_hand` 是一维夹爪关节组。命名状态 `open=0.8 rad`、`closed=0.05 rad` 是项目仿真
  预设，避开硬限位，不是厂家夹持宽度或夹持力标定。
- `rm65_arm_hand` 包含臂和夹爪，适合在关节空间里做集成规划；它不是一个七维笛卡尔 IK
  求解器。给这个组发布关节目标时应约束全部六个臂关节和夹爪主动关节。只约束单一臂关节，
  其余关节可能被规划器选到意外姿态。

旧的 `/manipulation/rm65b/target_pose` 入口仍只针对 `rm65_arm`，便于参数化末端位姿输入。
仿真夹爪用 MoveIt `FollowJointTrajectory` 映射到 Gazebo 控制器；实机则使用
`changingtek_gripper_driver` 内置的官方串口 SDK 包装。实机启动必须显式设置 `dry_run:=false`，
并核对串口、ID、波特率和读回单位。Gazebo `/joint_states` 是 RViz 当前姿态和 MoveIt 当前状态的来源；
不能把规划动画当成已执行。

## 为什么夹爪模式使用另一套物理 world

裸臂原 `empty.sdf` 使用 Gazebo 默认 DART 引擎。本次动态实验中，DART 下夹爪的一侧
从动关节停在 `-1 rad`，即使主动关节动作成功也没有同步。夹爪模式改用
`empty_mimic.sdf`，显式启用 Bullet Featherstone。随后从动关节位置随主动关节按原模型的
`+1/-1` 倍率变化；控制器和 MoveIt 集成执行均实测通过。裸臂 world 没有被更换。
Gazebo 文档说明 DART 是默认引擎、Bullet Featherstone 可单独选择；
`gz_ros2_control` 文档说明联动关节不应有命令接口，因此这里只为五个从动关节配置状态接口。

ROS1 CAD 碰撞网格在靠近闭合位时存在预期的指端接触或重叠。SRDF 只豁免了本次
PlanningScene 实际报告的相邻连杆、关节机构及指端内部碰撞对；这些豁免是仿真模型约定，
不构成物体抓取、接触力或防夹伤安全性验证。夹爪碰撞几何和硬件装配须单独复核。

参考：[Gazebo 8 物理引擎说明](https://gazebosim.org/api/sim/8/physics.html)、
[Humble gz_ros2_control 联动关节说明](https://control.ros.org/humble/doc/gz_ros2_control/doc/index.html)、
[知行 CTM2F110 产品参数](https://en.changingtek.com/diandong/159)。

## 实机迁移前必须补齐

核对夹爪铭牌中的 `E` 后缀、厂家机械图和法兰转接板实测变换、额定负载与质心、
手指/工装真实碰撞包络、固件及 RS485/Modbus 指令协议、急停链路、失联动作、
夹持力和速度限值，并完成低速有人监护的单独验收。CTM2F110 的 ROS 2 串口驱动已提供
`FollowJointTrajectory` action 和 Present Position 读回，但在上述条件完成前不得启用实机；
RM65B 实机驱动仍不在本仓库，不能把仿真双控制器配置直接用于 RM65B 实机。

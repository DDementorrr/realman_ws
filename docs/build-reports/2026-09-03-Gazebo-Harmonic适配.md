# Gazebo Harmonic 迁移构建报告

## 本次交付

新增四层项目自有实现：

- `robot_gazebo`：Panda/PR2 Harmonic 描述适配、controller YAML 和基础 world。
- `manipulation_moveit_adapter`：订阅 `geometry_msgs/PoseStamped` 目标，通过 MoveGroupInterface 规划并执行。
- `manipulation_bringup`：编排 Gazebo、时钟桥接、robot_state_publisher、controller、MoveIt、RViz 和目标 adapter。
- `manipulation_demo`：提供可重复输入 `x y z roll pitch yaw` 的交互发布器。

## Panda 执行数据流

```text
RViz / PoseStamped target
          ↓
      move_group
          ↓
FollowJointTrajectory + ParallelGripperCommand
          ↓
     gz_ros2_control
          ↓
   Gazebo joint state
          ↓
joint_state_broadcaster → robot_state_publisher/TF → RViz
```

RViz 不再产生一套独立的“仿真关节状态”；它消费 Gazebo controller 发回的 `/joint_states`，因此两个界面对同一次执行同步。

## PR2 适配策略

`moveit_resources_pr2_description` 仅提供已展开的 `robot.xml` 和 SRDF，不提供现代 PR2 MoveIt/Gazebo Harmonic 整机配置。其 URDF 中嵌入了 Gazebo Classic/ROS 1 扩展。当前适配采用：

1. `pr2_gz.urdf.xacro` 引用上游模型，注入 Harmonic `gz_ros2_control/GazeboSimSystem`。
2. `prepare_pr2_harmonic_urdf.py` 在启动时去除旧 Gazebo/ROS 1 插件和旧 transmission，不复制整份上游 URDF。
3. 为两个旧夹爪命令框架补充极小惯量，防止 URDF→SDF 转换器删除其非固定关节链。
4. 当前 controller 范围为左臂、右臂、头部和升降轴；底盘、传感器和夹爪执行器尚未现代化。

## 静态构建与转换结果

- `colcon build --symlink-install --packages-select ...`：5 个目标包构建成功。
- Panda Xacro 展开成功，`gz sdf -p` 转换成功。
- PR2 经适配器展开后约 76 KB，SDF 约 110 KB，`gz sdf -p` 返回 0 且 stderr 为 0 行。
- Panda/PR2 launch 参数解析成功。
- `manipulation_demo` 四元数转换测试：2/2 通过。
- 参数化目标运行回归：向 `/manipulation/target_pose` 发布可达目标，C++ adapter 规划约 0.024 s，MoveGroupInterface 返回 `Execute request success!`，adapter 返回 `Target reached successfully`。该测试使用 FakeSystem。
- Panda arm+hand mock 执行已端到端通过，详见同日实验报告。

## 安装与动态验收更新

用户已安装 `ros-jazzy-gz-ros2-control`，本机版本为 Jazzy apt 包 `1.2.19`。`scripts/verify_environment.sh` 的命令、MoveIt、`ros_gz`、TF2、controller_manager、trajectory controller 和 `gz_ros2_control` 检查现已全部通过。

首次动态启动暴露出两套 ros2_control URDF 共用 `/robot_description` 的集成缺陷：Gazebo controller_manager 接收了 MoveIt 的 `mock_components/GenericSystem` 描述，因而没有创建任何 Gazebo 硬件接口，三个 controller 均保持 inactive。现已完成以下修复：

- MoveIt/RViz 的 FakeSystem 描述保留在 `/robot_description`。
- GazeboSystem 描述由隔离的 description publisher 发布到 `/robot_description_gazebo`。
- `gz_ros2_control` 插件只订阅后者，并显式启用 `hold_joints`。
- Gazebo 以暂停状态生成模型；hardware 初始化并将 controller 配置为 inactive 后才解除暂停，随后原子激活 controller，避免重力在启动窗口内改变关节姿态。
- 新增 `scripts/verify_panda_gz_runtime.sh`，在联合环境运行时检查 hardware、controller、action server 和 `/joint_states`。

动态回归结果：`PandaGazeboSystem` 为 active，7 个臂接口与 1 个夹爪接口均 available/claimed，三个 controller 为 active；真实 Gazebo 后端上的参数目标规划及执行返回 `SUCCEEDED`，夹爪 `ParallelGripperCommand` action 也返回 `SUCCEEDED`。本次 Gazebo 联合仿真的 controller 动态验收由“阻塞”更新为“通过”。

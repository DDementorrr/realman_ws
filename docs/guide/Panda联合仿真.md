# MoveIt + RViz + Gazebo Harmonic 同步仿真指南

## 开始前

```bash
sudo apt update
sudo apt install ros-jazzy-gz-ros2-control
cd /home/ww/manipulation_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
/home/ww/scripts/verify_environment.sh
```

## 启动 Panda 联合环境

```bash
source /opt/ros/jazzy/setup.bash
source /home/ww/manipulation_ws/install/setup.bash
ros2 launch manipulation_bringup panda_moveit_gz.launch.py
```

启动顺序为 Gazebo 暂停 → 生成机器人 → 初始化 hardware → 配置 controllers → 解除暂停 → 激活 controllers → MoveIt/RViz/目标 adapter。该门控用于防止重力在 controller 接管前改变初始位姿。启动完成后应检查：

```bash
ros2 control list_controllers
ros2 action list -t
ros2 topic hz /joint_states
ros2 run tf2_ros tf2_echo world panda_hand
/home/ww/scripts/verify_panda_gz_runtime.sh
```

预期 `joint_state_broadcaster`、`panda_arm_controller`、`panda_hand_controller` 都为 `active`。

## 两份 robot description 的边界

联合启动有意维护两份运动学结构相同、控制后端不同的 URDF：

```text
/robot_description
  └─ MoveIt/RViz: mock_components/FakeSystem 描述

/robot_description_gazebo
  └─ Gazebo controller_manager: gz_ros2_control/GazeboSimSystem 描述
```

这不是重复的机器人状态源。真实执行状态只来自 Gazebo 的 `/joint_states`；两份描述分别告诉不同组件如何解释模型和加载 hardware plugin。不得让 Gazebo controller_manager 订阅 FakeSystem 描述，否则 hardware interface 为空、controller 无法激活，继而表现为关节下垂和 MoveIt current state 缺失。

## 输入方式 A：RViz 拖动

在 MotionPlanning 面板选择 `panda_arm`，拖动末端交互标记，然后点击 Plan 和 Execute。轨迹经 MoveIt 下发给 Gazebo controller；Gazebo 的关节反馈再通过 `/joint_states` 回到 RViz。

## 输入方式 B：交互式坐标

新开终端：

```bash
source /opt/ros/jazzy/setup.bash
source /home/ww/manipulation_ws/install/setup.bash
ros2 run manipulation_demo interactive_target --ros-args -p use_sim_time:=true
```

每行输入 `x y z roll pitch yaw`（米、弧度，默认坐标系 `panda_link0`），例如：

```text
0.45 0.00 0.45 3.14159 0.0 0.0
```

输入 `q` 退出。目标会发布到 `/manipulation/target_pose`，C++ adapter 负责计划和执行。运动期间新目标会被拒绝，避免多次执行相互覆盖。

也可直接发布单次参数化目标：

```bash
ros2 topic pub --once /manipulation/target_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: panda_link0}, pose: {position: {x: 0.45, y: 0.0, z: 0.45}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}"
```

## 同步性验收

1. 执行前记录 `/joint_states` 一次。
2. 用 RViz 或主题发送目标。
3. 观察 Gazebo 实体先按 controller 轨迹运动，RViz 随关节反馈更新。
4. 确认 adapter 输出 `Target reached successfully`，MoveIt 输出 execution `SUCCEEDED`。
5. 再记录 `/joint_states`，检查末态稳定且未超出 joint limits。

不要以 RViz 中的计划动画作为执行成功证据；必须同时检查 controller action 结果和 Gazebo 反馈。

## RViz 规划失败的排查顺序

1. 运行 `verify_panda_gz_runtime.sh`，先排除 hardware/controller/state 通路问题。
2. 确认 `/joint_states` 持续发布且 MoveIt 没有 `complete state ... not yet known` 持续告警。
3. 将目标放在初始末端附近，小幅移动并保持姿态，验证基础 IK。
4. 再检查目标是否超出工作空间、是否自碰撞，以及所选 planning group、end-effector link 和 planner 是否正确。
5. Plan 成功但 Execute 失败时，查看 controller action 类型、名称和轨迹起点容差；不要先修改 OMPL 参数。

主动 Ctrl-C 后出现 Gazebo `exit code -2` 或 move_group `exit code -15` 属于进程退出信号记录，不能作为规划失败的证据；应向前查找第一次 ERROR 以及 controller/hardware 状态。

## PR2 基础适配启动

```bash
ros2 launch manipulation_bringup pr2_gz.launch.py
```

这个 launch 用于验证 PR2 模型在 Harmonic 中的生成、双臂/上躯 controller 和 joint-state 通路。它同样使用规划/仿真双描述和暂停启动门控；历史 Gazebo Classic plugin 会在展开时移除。它不包含完整 PR2 MoveIt 配置，也不宣称底盘、夹爪和传感器已迁移。

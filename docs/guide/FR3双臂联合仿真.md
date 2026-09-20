# FR3 / 固定 FR3 Duo：Gazebo + MoveIt 使用指南

## 1. 一次性准备

```bash
cd /home/ww/manipulation_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
colcon build --symlink-install
source install/setup.bash
/home/ww/scripts/verify_environment.sh
```

上游版本由 `manipulation_jazzy.repos` 固定。如果重建干净工作空间：

```bash
mkdir -p /path/to/ws/src
cd /path/to/ws
vcs import src < /home/ww/manipulation_ws/manipulation_jazzy.repos
```

本工程已有项目包，`vcs import` 只恢复两个上游资源，不会恢复项目自有代码。

## 2. 启动单 FR3

```bash
source /opt/ros/jazzy/setup.bash
source /home/ww/manipulation_ws/install/setup.bash
ros2 launch manipulation_bringup franka_moveit_gz.launch.py variant:=fr3
```

启动器先保持世界暂停，依次完成 spawn、controller inactive load、解暂停、controller active，
最后才启动 MoveIt/RViz/target adapter。任一门控失败会停止整个 launch。

另开终端验收：

```bash
source /opt/ros/jazzy/setup.bash
source /home/ww/manipulation_ws/install/setup.bash
/home/ww/scripts/verify_franka_gz_runtime.sh fr3
```

在 RViz 中可拖动交互 marker，Plan 后 Execute。也可先读当前末端：

```bash
timeout 5 ros2 run tf2_ros tf2_echo base fr3_link8
```

然后只做小位移并保持原四元数：

```bash
ros2 topic pub --once /manipulation/fr3/target_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base}, pose: {position: {x: X, y: Y, z: Z}, orientation: {x: QX, y: QY, z: QZ, w: QW}}}"
```

## 3. 启动固定 FR3 Duo

```bash
ros2 launch manipulation_bringup franka_moveit_gz.launch.py variant:=fr3_duo
/home/ww/scripts/verify_franka_gz_runtime.sh fr3_duo
```

RViz planning group 可选择 `left_arm`、`right_arm` 或 `both_arms`。左右独立参数接口：

```text
/manipulation/left/target_pose
/manipulation/right/target_pose
```

独立接口收到 PoseStamped 后立即规划/执行对应单臂。

## 4. 双臂协调目标

协调接口使用不同话题，避免“发布左臂后立即运动、右臂尚未设置”的竞态：

```text
/manipulation/dual/left_target_pose
/manipulation/dual/right_target_pose
```

两个目标均发布后调用：

```bash
ros2 service call /manipulation/dual/plan_execute std_srvs/srv/Trigger '{}'
```

返回 `success=True, coordinated trajectory executed` 表示两个 goal 组成一次 `both_arms`
规划，且左右控制器都执行成功。

交互式输入：

```bash
ros2 run manipulation_demo interactive_dual_target --ros-args -p frame_id:=base
```

终端命令格式：

```text
left  x y z roll pitch yaw
right x y z roll pitch yaw
go
q
```

角度单位是弧度。建议先通过 `tf2_echo` 获取当前左右 link8 位姿，再做小步实验。

## 5. 启动参数

```bash
ros2 launch manipulation_bringup franka_moveit_gz.launch.py --show-args
```

- `variant`: `fr3` 或 `fr3_duo`；
- `planning_time`: MoveIt 规划时间；
- `velocity_scaling`：速度缩放，默认低速教学值；
- `acceleration_scaling`：加速度缩放。
- `planner_id`：参数化目标 adapter 使用的 OMPL planner，默认
  `RRTConnectkConfigDefault`；可选项和比较方法见
  [规划算法选择指南](规划算法选择.md)。

## 6. 判断同步是否真实

正确证据：

1. controller 均为 `active`；
2. position command interface 为 `claimed`；
3. Gazebo hardware state 为 `active`；
4. `/joint_states` 来自 joint_state_broadcaster；
5. MoveIt execution 返回 `SUCCEEDED`；
6. controller 日志返回 `Goal reached`。

RViz 规划动画、Gazebo 画面或 topic 发布成功都不能单独替代这些证据。

## 7. 常见问题

### 刚载入时关节下坠

应检查 launch 是否使用本指南的门控版本、Gazebo 是否起始暂停、ready initial_value 是否存在、
controller 是否在解暂停后立即激活。不要用 FakeSystem 描述启动 Gazebo。

### RViz 能规划但不能执行

```bash
ros2 control list_controllers
ros2 action list -t
ros2 control list_hardware_interfaces
```

对照 controller YAML 的 action namespace 和关节集合。先查执行链，不要先改 IK。

### 规划失败

先用当前 link8 姿态做 1–2 cm 平移，排除错误四元数/不可达目标；再查 start collision、TF、
planning group、关节限位和规划时间。

### RViz trajectory display 偶发 `No robot state or robot model loaded`

本轮观察到 RViz trajectory display 在执行瞬间偶发该显示层日志，但 RobotModel 已加载、
MoveIt/两个 controller 执行均成功、Gazebo 和 joint_states 正常。它不作为执行失败；若持续
影响显示，应重载 MotionPlanning display 或检查 RViz config，而不是修改控制链。

### Ctrl-C 后 Gazebo exit -2 / move_group SIGTERM

这是人工 SIGINT 关停时 launch 的退出码/超时升级，需与运行期进程自行死亡区别。判断故障要
看退出前是否有用户中断及执行结果。

## 8. 实机迁移警告

Gazebo 配置使用 position interface。Franka 官方固定 Duo 实机路径当前是 torque/effort；
切换实机不是替换一个 IP 地址，而是新增 hardware/controller adapter、实时配置、安全条件和
完整验收。请从官方 Jazzy 文档开始：

- https://github.com/frankarobotics/franka_ros2
- https://frankarobotics.github.io/docs/doc/franka_ros2_jazzy/franka_bringup/doc/index.html

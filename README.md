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
python3 scripts/verify_realman_gripper_gz_runtime.py
```

## Pick & Place：阶段 1（已知位姿场景）

沿用上述 RM65B、夹爪、控制器和 MoveIt 启动链路，只替换 Gazebo world，
并向 MoveIt PlanningScene 加入工作台与长方体。物体高 0.22 m，方便夹爪在
远离台面的高度接触物体；放置区域是工作台上的绿色标记，
`/object_pose` 和 `/place_pose` 是 `world` 坐标系下的已知位姿。

```bash
source /opt/ros/humble/setup.bash
source .third_party/gz_ros2_control/install/setup.bash
source install/setup.bash
ros2 launch pick_place_demo known_scene.launch.py with_gripper:=true
```

无图形服务器加 `headless:=true use_rviz:=false`。保持仿真运行，在第二终端执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
bash scripts/verify_realman_gz_runtime.sh
python3 scripts/verify_realman_gripper_gz_runtime.py
python3 scripts/verify_pick_place_scene.py
python3 scripts/check_pick_place_ik.py
```

检查脚本同时读取 Gazebo 模型位姿、MoveIt 碰撞几何和 ROS 2 已知位姿；
IK 预检要求抓取姿态除夹爪与目标物体接触外不与台面相交，并要求放置姿态无碰撞。
单看 RViz 中的物体或机械臂不算通过。当前这一步只验证场景与原控制链路，
不会自动发起抓取或移动物体。场景尺寸与位姿在
`src/ros2_adapters/pick_place_demo/config/known_scene.yaml` 中配置；修改后需同步
`src/realman_rm65/robot_gazebo/worlds/rm65b_pick_place.sdf` 并重新运行检查。

## Pick & Place：阶段 2（MTC 任务规划，已运行验收）

网络可用时安装 Humble 的上游 MTC 包，再以严格依赖检查构建：

```bash
sudo apt install ros-humble-moveit-task-constructor-core \
  ros-humble-moveit-task-constructor-capabilities \
  ros-humble-moveit-task-constructor-visualization
source /opt/ros/humble/setup.bash
colcon build --packages-select pick_place_mtc --cmake-args -DPICK_PLACE_REQUIRE_MTC=ON
source install/setup.bash
ros2 pkg executables pick_place_mtc  # 应包含 plan_pick_place
```

下面的独立启动文件会复用阶段 1 的完整仿真启动：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch pick_place_mtc plan_pick_place.launch.py
```

无图形服务器加 `headless:=true use_rviz:=false`。

节点等待 `/object_pose`、`/place_pose` 和 MoveIt 碰撞场景就绪，复用现有
`ready`、`open` 状态，并为本场景设置夹爪闭合目标，生成 Home、预抓取、接近、闭合、附着、抬升、
搬运、下降放置、打开、分离、撤退、返回 Home 的完整 MTC 方案。自由空间段使用
OMPL，接近、抬升、放置和撤退使用 MTC CartesianPath。阶段 2 只发布规划解，
不执行；完整规划已在运行中的 Gazebo + MoveIt 上验收通过。2026-09-24 的本地
严格编译使用忽略目录 `.third_party/mtc/debroot` 中的 Humble MTC 二进制包，
并不等同于系统安装。

规划节点正常输出 `/pick_place/plan_ready=true` 后，在第二终端运行
`python3 scripts/verify_mtc_stage2.py`。该检查还确认物体仍在 MoveIt 的 world 中，
防止把仅完成规划误判为 Gazebo 抓取执行。

## Pick & Place：阶段 3（固定关节仿真抓取）

独立启动阶段 3；该入口才启用 Gazebo 抓取锚点及 MTC 执行能力，阶段 1、2
的启动入口保持规划或场景验证模式。若 MTC 安装在本地忽略目录而非系统目录，
启动和验证终端都必须加载同一环境，推荐直接 source 仓库脚本：

```bash
source scripts/source_pick_place_env.sh
```

```bash
ros2 launch pick_place_sim execute_pick_place.launch.py headless:=true use_rviz:=false auto_execute:=true
```

第二终端运行 `python3 scripts/verify_pick_place_stage3.py`，检查完整动作成功、
Gazebo 固定关节已释放、Gazebo 实际物体位姿接近放置目标，以及 MoveIt
PlanningScene 中物体回到 world。`auto_execute:=false` 可在规划完成后手动调用
`ros2 service call /pick_place/execute std_srvs/srv/Trigger '{}'`。一轮完成后可调用
`ros2 service call /pick_place/reset std_srvs/srv/Trigger '{}'`，再调用执行服务；
同一仿真进程内的第二轮已通过上述验收脚本。这一阶段使用独立的
`rm65b_pick_place_fixed.sdf`：零重力且目标物体无碰撞，仅用于验证固定附着与
规划场景状态同步，**不代表物理接触抓取**。夹爪闭合时动态加载 Gazebo
DetachableJoint，避免插件启动时默认附着使物体偏离初始位置。阶段 4 的真实接触
抓取、RGB-D 相机和 ArUco 视觉定位均已完成独立运行时验收。

## Pick & Place：阶段 4（真实接触抓取）

`physical_pick_place.launch.py` 是独立试验入口：使用有重力、有物体碰撞的
`rm65b_pick_place_physical.sdf`、单独的接触摩擦和仿真夹爪控制器参数；
MTC 仍走原有任务接口，但夹爪绕接近轴旋转 90°，抓取目标沿工具轴偏移
20 mm，并将预抓取开度设为 1.0 rad。该场景使用 50×70×220 mm、30 g 的
轻质试验物体；仅在此模式中移除内指网格碰撞，让可替换垫片成为主要接触面，
视觉、惯量与其他阶段的描述保持不变。物体上的 Gazebo 接触传感器经
`/pick_place/sim/contact_state` 暴露接触对象；运行时验收要求搬运期间双垫片
同时接触。物理模式的放置 Cartesian 下移为 1–2 cm，释放目标高于桌面 2 cm，避免原 8–12 cm
下移把物体压进桌面。不代表任意负载均能稳定抓取。
该入口**不创建也不调用 Gazebo DetachableJoint**。默认不自动执行：

```bash
ros2 launch pick_place_sim physical_pick_place.launch.py headless:=true use_rviz:=false
python3 scripts/verify_mtc_stage2.py  # 在另一终端确认完整规划
python3 scripts/verify_pick_place_stage4.py  # 另一终端，执行前启动
ros2 service call /pick_place/execute std_srvs/srv/Trigger '{}'
```

当前 50×70×220 mm 双垫片接触模型在三次全新仿真进程中完成纯接触抓取、
搬运、放置并返回 Home；其中两次由 `verify_pick_place_stage4.py` 独立验收，
双垫片搬运接触样本分别为 524 和 542，最终落点误差分别为 9 mm 和 11 mm，
Gazebo 物体直立且 PlanningScene 同步。圆形放置区域不约束物体偏航，
因此运行时判据区分绕竖直轴旋转与倾倒。
必须以 Gazebo 物体实际位姿、姿态和 MoveIt PlanningScene 同步结果确认完整循环，
不能把 MTC 规划成功或关节动作完成当作物理抓取成功。此结果仅覆盖上述轻质
试验物体，并不证明夹爪对其他质量或形状也稳定。

## Pick & Place：阶段 5（Gazebo Harmonic RGB-D 相机）

`rgbd_pick_place.launch.py` 复用阶段 4 场景和任务接口，在 Gazebo 中单独生成
顶视原生 RGB-D 传感器；不自动执行机械臂动作。`ros_gz_bridge` 输出与 RealSense
命名接近的 `/camera/camera/color/image_raw`、
`/camera/camera/depth/image_rect_raw`、两路 `camera_info` 和
`/camera/camera/depth/color/points`。光学帧为 `camera_color_optical_frame`，
通过 TF 连到 `base_link`。在已构建工作空间的终端运行：

```bash
ros2 launch pick_place_sim rgbd_pick_place.launch.py headless:=true use_rviz:=false
python3 scripts/verify_pick_place_stage5.py  # 另一终端
```

阶段 5 验收检查实际 640×480 RGB、浮点深度、相机内参、有效点云、时间戳和
`base_link` 变换；该传感器是仿真相机，并不等同于真实 RealSense 标定。

## Pick & Place：阶段 6（ArUco 视觉定位与统一入口）

`vision_pick_place.launch.py` 使用同一仿真场景和 RGB-D 话题，启动独立的
`aruco_object_pose` 节点。它检测物体顶部的 4x4、ID 0 标记，以深度和
CameraInfo 反投影得到标记中心，再按已知物体高度恢复物体中心，发布
`base_link` 下的 `/object_pose`，并以 `CollisionObject.ADD` 更新 MoveIt
PlanningScene。MTC 仍只依赖 `/object_pose`，因此真实 RealSense 节点可以
替换该视觉节点而不修改任务规划器。

规划发布 `plan_ready` 后，视觉输入和 PlanningScene 更新冻结到本次任务的
初始观测，避免执行中的物体运动反过来修改已规划任务；下一次启动会重新定位。
完整自动入口：

```bash
ros2 launch pick_place_sim vision_pick_place.launch.py headless:=true use_rviz:=false
python3 scripts/verify_pick_place_stage6.py  # 规划-only 视觉/PlanningScene 门槛
```

最终自动闭环已完成无固定附着的抓取、搬运、放置、释放、撤退和 Home 返回；
该物理结果仍限定于当前 30 g、50x70x220 mm 试验物体。

## 迁移范围

具体判定见 [迁移清单](docs/迁移记录/迁移清单.md) 和
[迁移报告](docs/迁移记录/2026-09-20-ROS2_Humble_RM65B夹爪迁移报告.md)。
其中明确区分项目自建内容、只记录来源的官方内容、教学算法和不应迁移的生成目录。

本仓库仅做 Gazebo 和 MoveIt 仿真。没有连接实机、没有启用串口/Modbus，并且
CTM2F110E 与仓库中 CTM2F110 模型的等同性、法兰安装变换和真实负载参数尚未确认。

## 工程文档库

主工作空间的项目文档已完整同步到 [`docs/`](docs/)，包括构建报告、实验记录、使用指南、
机器人参数说明、教材章节和原始系统说明 PDF。建议按以下入口阅读：

- [指南索引](docs/guide/索引.md)：环境、架构、仿真和 MoveIt 使用说明；
- [实验记录索引](docs/experiments/索引.md)：可复现的验证过程与结果；
- [构建报告索引](docs/build-reports/索引.md)：工程变更和构建状态；
- [机器人资料索引](docs/robots/索引.md)：各机械臂和夹爪的参数资料；
- [教材索引](docs/textbook/教材索引.md)：运动学、规划、控制和双臂系统学习路线。

`docs/迁移记录/` 保留本仓库面向 Ubuntu 22.04、ROS 2 Humble 和 Gazebo Harmonic 的迁移
清单与报告；其他官方上游资产仍以 `upstream/*.repos` 和相应来源记录为准。

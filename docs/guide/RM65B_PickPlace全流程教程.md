# RM65B Pick & Place 全流程教程

本文用于在有图形桌面上复现完整流程、直接观察 Gazebo/RViz 中的运动，并验证 RGB-D 与视觉定位接口。所有终端都应在 `/home/ww/realman_ws` 下操作。

## 0. 先加载统一环境

阶段 2/3/4/5/6 都依赖本地 Humble MTC 二进制前缀。未加载该前缀时，常见现象是：

- `plan_pick_place` 报 `librviz_marker_tools.so: cannot open shared object file`；
- `execute_pick_place.py` 报 `No module named moveit_task_constructor_msgs`；
- 验证脚本只能看到场景或相机话题，看不到完整任务和执行状态。

在每一个新终端执行：

```bash
cd /home/ww/realman_ws
source scripts/source_pick_place_env.sh
```

脚本会依次加载 ROS 2 Humble、源码构建的 `gz_ros2_control`、工作空间和本地 MTC。可用下面命令确认：

```bash
ros2 pkg executables pick_place_mtc
python3 -c 'from moveit_task_constructor_msgs.action import ExecuteTaskSolution; print("MTC Python OK")'
ldd install/pick_place_mtc/lib/pick_place_mtc/plan_pick_place | grep 'not found' || true
```

最后一条不应输出任何 `not found`。

## 1. 有界面启动和观察窗口

公共 `pick_place_mtc plan_pick_place.launch.py` 只启动外部配置的规划节点，必须传入
`planning_config:=/path/to/moveit.yaml task_config:=/path/to/task.yaml`。它不负责 Gazebo、场景或执行器。
完整仿真通过 `pick_place_sim` 编排。需要观察 Gazebo GUI 和 RViz2 时使用 `headless:=false use_rviz:=true`：

```bash
ros2 launch pick_place_sim execute_pick_place.launch.py \
  headless:=false use_rviz:=true auto_execute:=false
```

每次只运行一套仿真。若上一轮 launch 异常中断，先检查并结束残留进程；否则旧 Gazebo
会继续占用 `/world/empty`，新一轮控制器可能报告 `Controller already loaded`，随后 launch
主动关闭 Gazebo。此时 Gazebo 的 `exit code -15` 是清理信号，不是首发故障：

```bash
ps -ef | grep -E '[g]z sim|[r]os2 launch' || true
# 仅对确认属于本工作空间旧实例的 PID 执行：
kill -TERM <旧的gz或launch-PID>
```

重新启动后，先确认日志中没有 `Failed to configure controller`，再运行阶段验证脚本。

Gazebo 用于观察真实仿真模型和关节反馈；RViz2 的 MotionPlanning、RobotModel、PlanningScene 用于观察规划场景和规划轨迹。RViz 的绿色/动画轨迹不能代替执行证据，执行是否完成以 Gazebo `/joint_states`、控制器 action 和 `/pick_place/execution_status` 为准。

阶段 3 是最适合第一次直观看全流程的入口：它使用固定/可拆附着，动作可重复，不受物理摩擦随机性影响。规划完成后，在第二终端执行：

```bash
source scripts/source_pick_place_env.sh
python3 scripts/verify_mtc_stage2.py
ros2 service call /pick_place/execute std_srvs/srv/Trigger '{}'
python3 scripts/verify_pick_place_stage3.py
```

也可以直接在启动命令中加 `auto_execute:=true`，但建议第一次使用手动 service，便于在 Gazebo 中确认 Home、接近、闭合、抬升、搬运、放置、打开和返回 Home 的每个动作。

视觉腕部相机模式会先完成观察动作，再等待图像定位和 MTC 规划。`auto_execute:=false` 时，
手动触发只是延后执行，不改变 MTC 起点；规划从观测动作后的 `CurrentState` 直接连接到
预抓取位，不会先回初始化 `ready` 姿态。放置、松爪和撤退完成后仍会返回 Home。

复位后才能重新执行：

```bash
ros2 service call /pick_place/reset std_srvs/srv/Trigger '{}'
```

## 2. 阶段 4：真实接触抓取

阶段 4 不向 Gazebo 发送固定附着命令，而是使用夹爪碰撞、摩擦、质量和接触：

```bash
ros2 launch pick_place_sim physical_pick_place.launch.py \
  headless:=false use_rviz:=true auto_execute:=false grip_target:=0.40
```

先在第二终端验证规划，并启动覆盖完整搬运过程的检查；保持它运行：

```bash
source scripts/source_pick_place_env.sh
python3 scripts/verify_mtc_stage2.py
python3 scripts/verify_pick_place_stage4.py
```

再在第三终端加载同一环境并触发执行：

```bash
source scripts/source_pick_place_env.sh
ros2 service call /pick_place/execute std_srvs/srv/Trigger '{}'
```

不要执行结束后才启动阶段 4 检查，否则无法采到搬运期间的双侧接触及关节状态。

这里必须分别观察：

- `/pick_place/sim/contact_state` 是否包含左右指垫接触；
- `/pick_place/sim/attachment_state` 是否保持 `detached`（真实接触模式不应把它当作成功抓取证据）；
- 搬运后物体是否仍跟随夹爪，最终是否落在放置区。

如果日志出现 `Object slipped in the gripper`，这是物理抓取失败，不是 MTC 规划失败。先用阶段 3 验证上层任务和控制器，再调整 `grip_target`、物体质量/摩擦或运动速度。

阶段 6 可以直接执行视觉定位驱动的抓取任务：ArUco 节点发布 `/object_pose`，MTC 订阅该位姿并生成任务，执行节点在收到完整解后可以自动执行或等待手动触发。但 `verify_pick_place_stage6.py` 只验证视觉位姿和 PlanningScene 同步，不证明抓取、搬运、放置已经成功。阶段 6 默认是 `auto_execute:=true`、真实接触抓取；首次验收建议关闭自动执行，并用固定附着把定位/MTC/控制器链路与接触摩擦问题分开：

```bash
ros2 launch pick_place_sim vision_pick_place.launch.py \
  headless:=false use_rviz:=true auto_execute:=false \
  world_file:=/home/ww/realman_ws/install/robot_gazebo/share/robot_gazebo/worlds/rm65b_pick_place_physical.sdf \
  pick_place_attachment:=true physical_grasp:=false physical_contact:=false
```

该命令打开 Gazebo GUI 和 RViz。相机识别到 marker 后会持续发布位姿并更新 PlanningScene；MTC 在拿到目标位姿和放置位姿后规划。确认 `plan_ready` 为 `true`、阶段 6 验证脚本通过且 Gazebo 中目标位置合理后，在终端 2 手动启动动作：

```bash
ros2 service call /pick_place/execute std_srvs/srv/Trigger '{}'
ros2 topic echo /pick_place/execution_status
ros2 topic echo /pick_place/execution_success
python3 scripts/verify_pick_place_stage3.py
```

阶段 3 verifier 使用 `/place_pose` 中的目标，检查执行成功、Gazebo 最终放置误差不超过 4 cm，以及 Gazebo 和 PlanningScene 均已解除附着；它不依赖专用场景路径，可用于这里的视觉固定附着回归。不要把 `verify_pick_place_stage6.py` 的 PASS 当作动作成功。确认固定附着全流程后，再以 `pick_place_attachment:=false physical_grasp:=true physical_contact:=true` 测真实接触；接触失败需要结合实际接触、位姿和执行日志诊断。

注意：视觉节点收到 `/pick_place/plan_ready=true` 后冻结当前检测位姿，避免执行中目标估计变化导致场景与已规划轨迹不一致。因此一次执行后如需再次从新图像定位并执行，先结束整套 launch，再重新启动；仅调用 `/pick_place/reset` 不会解除视觉节点的冻结状态。

## 3. RGB-D 相机接口和图像观察

启动 RGB-D 场景：

```bash
ros2 launch pick_place_sim rgbd_pick_place.launch.py \
  headless:=false use_rviz:=true auto_execute:=false
```

接口映射如下：

| 语义 | 话题 | 类型 | 预期 |
|---|---|---|---|
| 彩色图像 | `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | `640x480`, `rgb8` |
| 深度图像 | `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | `640x480`, `32FC1` |
| 彩色内参 | `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 与彩色图像同帧 |
| 深度内参 | `/camera/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 与深度图像同帧 |
| 点云 | `/camera/camera/depth/color/points` | `sensor_msgs/msg/PointCloud2` | `640x480` |
| 光学帧 | `camera_color_optical_frame` | TF | 可变换到 `base_link` |

命令行检查：

```bash
ros2 topic list | grep -E 'camera|points'
ros2 topic info -v /camera/camera/color/image_raw
ros2 topic echo /camera/camera/color/camera_info --once
ros2 topic hz /camera/camera/color/image_raw
python3 scripts/verify_pick_place_stage5.py
```

直观看图像可使用：

```bash
ros2 run rqt_image_view rqt_image_view
```

在窗口中选择 `/camera/camera/color/image_raw` 或 `/camera/camera/depth/image_rect_raw`。也可以在 RViz2 添加 `Image` display，分别选择彩色和深度话题；添加 `PointCloud2` display 后选择点云话题，Fixed Frame 设为 `base_link` 或 `camera_color_optical_frame`。

阶段 5 的时间戳来自 Gazebo RGB-D 消息，经 `ros_gz_bridge` 转为 ROS 消息。`verify_pick_place_stage5.py` 同时检查消息存在、编码、尺寸、内参、有效深度、点云、光学 TF 和时间戳同步。检查端在各话题最多 30 条消息的缓存中寻找时间匹配的数据组，避免把 GUI 负载下不同时刻到达的首条 RGB/Depth/点云消息直接比较。120 秒内仍没有时间差不超过 0.5 秒且时间戳非零的数据组则失败；这个检查阈值不代表视觉允许 0.5 秒误差，视觉节点的 RGB/Depth 配对仍限制为 50 ms。

## 4. 视觉定位与 `/object_pose`

阶段 6 会在 RGB-D 场景上启动 ArUco 节点：

```bash
ros2 launch pick_place_sim vision_pick_place.launch.py \
  headless:=false use_rviz:=true auto_execute:=false grip_target:=0.40
```

视觉链路为：

1. 彩色图像检测 `DICT_4X4_50` 的配置 marker（默认侧面 ID 1，也可用启动参数选择顶面 ID 0）；
2. 从深度图像读取 marker 中心深度；
3. 使用 `CameraInfo` 内参反投影到相机光学坐标系；
4. 用 TF 将点变换到 `base_link`；
5. 减去物体高度的一半得到物体中心；
6. 发布 `/object_pose`，并通过 `ApplyPlanningScene` 更新 `pick_object`；
7. MTC 只消费 `/object_pose` 和 `/place_pose`，不依赖 ArUco 实现细节。

### 4.1 分步验证视觉数据链

在每条命令前确认当前终端已 source 统一环境。先检查节点和发布者，确认启动的是唯一一套仿真，避免旧进程造成 QoS 或重复发布者冲突：

```bash
ros2 node list | sort
ros2 topic list | grep -E '^/camera/|^/object_pose$|^/place_pose$|^/pick_place/'
ros2 topic info -v /camera/camera/color/image_raw
```

依次验证图像、深度、内参和点云是否持续发布：

```bash
ros2 topic type /camera/camera/color/image_raw
ros2 topic hz /camera/camera/color/image_raw
ros2 topic echo /camera/camera/color/image_raw --once
ros2 topic hz /camera/camera/depth/image_rect_raw
ros2 topic echo /camera/camera/depth/image_rect_raw --once
ros2 topic echo /camera/camera/color/camera_info --once
ros2 topic echo /camera/camera/depth/camera_info --once
ros2 topic hz /camera/camera/depth/color/points
```

图像消息重点看 `header.stamp`、`header.frame_id`、`height`、`width`、`encoding`。当前期望彩色为 `rgb8`，深度为 `32FC1`，尺寸均为 `640x480`。再用 `rqt_image_view` 选择彩色和深度话题，肉眼确认所选 marker 清晰可见且没有被机械臂/桌面遮挡。阶段 5 verifier 检查 RGB-D 接口与两类相机时间戳；它不检查 ArUco 是否检测到 marker。

确认视觉节点订阅及输出话题 QoS：

```bash
ros2 node info /aruco_object_pose
ros2 topic info -v /object_pose
ros2 topic echo /object_pose --once --qos-durability transient_local --qos-reliability reliable
```

`/object_pose` 类型是 `geometry_msgs/msg/PoseStamped`，当前发布端采用 reliable + transient-local，`header.frame_id` 应为 `base_link`。如果 `ros2 topic info -v` 显示多个发布者，先关闭遗留 launch。遇到 `incompatible QoS ... DURABILITY` 时，检查输出中每个 publisher/subscription 的 durability；不要仅凭 topic 存在判断已经收到消息。可用下面的 TF 命令核对相机坐标到机器人基座的变换：

```bash
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

阶段 6 的定位计算是：检测 marker 中心像素 `(u,v)`，从当前缓存深度图取该像素深度 `Z`，用相机内参 `fx, fy, cx, cy` 反投影得到相机坐标 `(X,Y,Z)`，再经 TF 转到 `base_link`，按配置的物体高度减去半高得到物体中心。当前拒绝 RGB 与深度时间戳相差超过 50 ms 的图像，使用 RGB 消息时间戳查询 TF，并将该时间戳写入 `/object_pose`。这仍是最新深度缓存配对；移动相机的进一步验证需覆盖时间配对、运动模糊和标记遮挡。

### 4.2 验证 PlanningScene、规划和执行

先验证视觉位姿及 PlanningScene 对象一致：

```bash
ros2 topic echo /object_pose --once
ros2 topic info -v /object_pose
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
python3 scripts/verify_pick_place_stage6.py
```

正确结果应满足：`/object_pose.header.frame_id == base_link`，位置非零，PlanningScene 中存在 `pick_object`，且视觉位姿与 PlanningScene 位置误差小于检查阈值。接着检查 MTC 的输入和输出：

```bash
ros2 topic info -v /place_pose
ros2 topic echo /place_pose --once
ros2 topic echo /pick_place/plan_ready --once
ros2 topic echo /pick_place/task_description --once
ros2 topic info -v /pick_place/solution
```

只有 `plan_ready: true` 且 solution/task description 已发布，才能认为完整任务规划就绪。建议阶段 6 首轮使用 `auto_execute:=false`，确认 RViz/Gazebo 里的目标与规划合理后再执行。真实接触模式先在单独终端运行 `python3 scripts/verify_pick_place_stage4.py`；固定附着模式改用 `python3 scripts/verify_pick_place_stage3.py`。保持检查运行，再在另一个终端触发：

```bash
ros2 service call /pick_place/execute std_srvs/srv/Trigger '{}'
ros2 topic echo /pick_place/execution_status
ros2 topic echo /pick_place/execution_success
ros2 topic hz /joint_states
ros2 topic echo /pick_place/sim/attachment_state
ros2 topic echo /pick_place/sim/object_pose
```

执行时 Gazebo 窗口应能看到机器人关节实际运动；`/joint_states` 应持续更新，执行状态应经过各段并最终报告成功。固定附着模式下还应观察 attachment 从抓取后 attached，释放/收尾后 detached，物体最终位姿落入放置区域。真实接触模式不要用 attachment 状态作为抓取成功证据，应看夹爪接触和搬运后的 Gazebo 物体位姿。规划成功只代表 MTC 已生成解，不代表物体已经完成抓取。

## 5. MTC 任务接口和执行状态

MTC 节点 `pick_place_mtc/plan_pick_place` 将任务拆为：

```text
CurrentState
  -> open gripper
  -> Pre-Grasp (OMPL)
  -> Approach (Cartesian)
  -> Close Gripper
  -> Attach in PlanningScene
  -> Lift (Cartesian)
  -> Transport (OMPL)
  -> Place (Cartesian)
  -> Open Gripper
  -> Detach in PlanningScene
  -> Retreat (Cartesian)
  -> return Home
```

视觉模式下 `CurrentState` 采样腕部相机观察动作后的机器人状态；Home 只保留为任务末尾的
回收姿态，不插入抓取前轨迹。

接口分类：

| 接口 | 类型 | 作用 |
|---|---|---|
| `/object_pose` | `PoseStamped` | 目标物体位姿；可来自已知场景或视觉 |
| `/place_pose` | `PoseStamped` | 放置目标位姿 |
| `/pick_place/task_description` | MTC `TaskDescription` | 任务阶段和 stage ID |
| `/pick_place/solution` | MTC `Solution` | 规划出的子轨迹序列 |
| `/pick_place/plan_ready` | `Bool` | 完整 MTC 解已发布 |
| `/pick_place/execute` | `std_srvs/srv/Trigger` | 执行一次已发布方案 |
| `/pick_place/reset` | `std_srvs/srv/Trigger` | 重置 Gazebo 物体和 PlanningScene |
| `/pick_place/execution_status` | `std_msgs/msg/String` | 当前执行阶段和失败原因 |
| `/pick_place/execution_success` | `Bool` | 一轮执行最终结果 |
| `/joint_states` | `sensor_msgs/msg/JointState` | Gazebo 真实关节反馈 |

检查顺序建议是：

```bash
ros2 topic echo /pick_place/plan_ready --once
ros2 topic echo /pick_place/task_description --once
ros2 topic echo /pick_place/execution_status
ros2 topic echo /pick_place/sim/contact_state
ros2 topic echo /pick_place/sim/object_pose
ros2 topic echo /joint_states
```

如果看到 `New publisher discovered ... incompatible QoS ... DURABILITY`，先用 `ros2 topic info -v /object_pose` 查看发布者。当前已知场景和视觉节点统一使用 reliable + transient-local；如果仍有旧进程残留，应先关闭旧的 ROS 2/Gazebo 进程后重新启动。

## 6. 推荐的全流程验收顺序

1. 先用阶段 3 + `headless:=false` 验证能在 Gazebo/RViz 中看到完整动作，并通过 `verify_pick_place_stage3.py`。
2. 再用阶段 4 验证真实接触和重复抓取；物理滑落只归类为接触参数问题。
3. 启动阶段 5，先在 `rqt_image_view` 中确认彩色/深度画面，再运行 `verify_pick_place_stage5.py`。
4. 启动阶段 6，依次检查相机消息、marker 检测后的 `/object_pose`、PlanningScene、MTC `plan_ready`；再调用 `/pick_place/execute`，从 execution status 和 Gazebo 反馈确认动作完成。
5. 最后才使用 `auto_execute:=true` 做连续回归；每轮失败后先调用 `/pick_place/reset`，不要在执行中重启视觉节点或重复发布目标位姿。

## 7. 当前配置与验收边界

腕部相机盒体中心固定在 `Link6` 局部 `+Y 0.10 m`，仅用于显示，不加入碰撞几何。观察姿态下该方向约对应世界 `+Z 0.10 m`；基座到相机的完整 TF 仍随腕部姿态变化。默认视觉 marker 是朝向相机的侧面 ID 1，顶部 ID 0 可通过启动参数选择。

视觉启动会先执行观察位，再发布 `/pick_place/observation_ready` 并进行定位。观察位只是启动阶段的工具目标；MTC 抓取规划从观察动作后的 `CurrentState` 开始，手动执行时不会先回初始化 `ready`，完成放置和撤退后才返回 Home。

首次验收建议关闭自动执行：

```bash
source scripts/source_pick_place_env.sh
ros2 launch pick_place_sim vision_pick_place.launch.py \
  headless:=false use_rviz:=true wrist_camera:=true auto_execute:=false \
  observation_xyz:='[0.36, -0.18, 0.075]' \
  observation_quaternion:='[0.5, 0.5, 0.5, 0.5]'
```

在另一个终端检查观察到位和图像：

```bash
# 必须为 true：运动成功后才发布，支持后启动的检查端读取。
timeout 30s ros2 topic echo /pick_place/observation_ready std_msgs/msg/Bool \
  --qos-durability transient_local --once
# 确认目标和完整侧面 marker 可见。
ros2 run rqt_image_view rqt_image_view /camera/camera/color/image_raw
```

图像窗口保持打开，再执行：

```bash
python3 scripts/probe_pick_place_marker.py
python3 scripts/verify_pick_place_stage5.py --wrist-camera
python3 scripts/verify_pick_place_stage6.py
ros2 topic echo /object_pose geometry_msgs/msg/PoseStamped \
  --qos-durability transient_local --once
```

marker probe 只证明可见性；阶段 5/6 还要证明 RGB-D、动态 TF、视觉位姿和 PlanningScene 同步。调整观察姿态后必须重新启动本轮环境并重新检查，不能沿用旧的 latched ready 或规划。

### 7.1 物理接触抓取验收

默认物理模型使用独立的指垫 box collision、指垫和物体接触摩擦系数 `1.2` 及低夹持 effort，
未配置显式黏附力。首次验收建议关闭自动执行，先确认规划和视觉定位，再观测接触搬运：

```bash
source scripts/source_pick_place_env.sh
ros2 launch pick_place_sim vision_pick_place.launch.py \
  headless:=false use_rviz:=true wrist_camera:=true auto_execute:=false \
  grip_target:=0.40 pad_compliance:=false
```

另一个终端检查，按顺序执行；规划可能需要数分钟：

```bash
source scripts/source_pick_place_env.sh
python3 scripts/verify_mtc_stage2.py --timeout 900
python3 scripts/verify_pick_place_stage5.py --wrist-camera
python3 scripts/verify_pick_place_stage6.py
# 必须在触发之前启动，保留本终端观察完整过程。
python3 scripts/verify_pick_place_stage4.py
```

确认上述规划/定位检查均通过、阶段 4 verifier 正在等待后，在第三终端触发：

```bash
source scripts/source_pick_place_env.sh
ros2 service call /pick_place/execute std_srvs/srv/Trigger '{}'
```


MoveIt 中的 mimic 关系用于理想连杆规划；反馈 TF 使用 `/joint_states` 的 Gazebo 实测角度，反馈模型通过 `/robot_description_feedback` 发布。因接触约束产生的实际连杆偏差不会再被 TF 中的理想 mimic 关系覆盖。运动过程中另开终端执行只读检查：

```bash
python3 scripts/verify_gripper_feedback_tf.py --duration 120
```

应输出 `PASS measured gripper TF`。阶段 4 verifier 还要求搬运期双侧指垫接触、物体放置和 PlanningScene 同步。`pad_compliance:=true` 只是可选弹簧近似，不是默认配置，也不等同真实橡胶。

逐轮评分、姿态调参、截图和失败样本归档在[构建报告](../build-reports/索引.md)与[实验记录](../experiments/索引.md)，不在教程中重复维护。阶段门槛见 [RM65B Pick & Place 长时目标任务分类](RM65B_PickPlace长时目标任务分类.md)。

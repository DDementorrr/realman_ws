# RM65B 仿真操作与接口速查

Ubuntu 22.04 · ROS 2 Humble · Gazebo Harmonic。流程解释、场景差异、验收边界见 [最新工作流说明](docs/guide/RM65B仿真工作流与接口说明.md)；[指南索引](docs/guide/索引.md)。以下运动指令只用于仿真。

实机：[测试与标定清单](docs/guide/RM65B实机测试与标定清单.md) · [准备状态记录](docs/experiments/2026-10-28-RM65B实机准备状态与验收记录.md)。

## 1. 环境与构建

每个终端都先执行；同一轮使用相同 `ROS_DOMAIN_ID`、`GZ_PARTITION`。

```bash
cd ~/realman_ws
source scripts/source_pick_place_env.sh
# 仅增量安装自定义 RViz 面板，不修改 RViz/MoveIt 源码：
bash scripts/build_operator_panel.sh
source scripts/source_pick_place_env.sh
```

中文显示为方框时，执行后重新启动 RViz：

```bash
bash scripts/setup_operator_panel_fonts.sh
```

新机器首次安装：

```bash
cd ~/realman_ws
bash scripts/setup_ubuntu2204_ros2_humble.sh
bash scripts/import_upstream.sh
bash scripts/prepare_realman_model_source.sh
bash scripts/build_gz_ros2_control_harmonic.sh
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys="ros_gz ros_gz_bridge ros_gz_sim gz_ros2_control"
sudo apt install fonts-noto-cjk ros-humble-moveit-task-constructor-core \
  ros-humble-moveit-task-constructor-capabilities ros-humble-moveit-task-constructor-visualization
source /opt/ros/humble/setup.bash
source .third_party/gz_ros2_control/install/setup.bash
colcon build --symlink-install
source install/setup.bash
python3 scripts/verify_migration_manifest.py
bash scripts/verify_environment.sh
```

## 2. 启动场景（一次只选一个）

```bash
# 基础机械臂+夹爪，无抓放任务：
ros2 launch manipulation_bringup realman_moveit_gz.launch.py with_gripper:=true
# 桌面与已知物体，适合面板、拖动、终端 A→B：
ros2 launch pick_place_demo known_scene.launch.py with_gripper:=true
# 固定附着完整抓放；默认不执行，等待独立任务接口：
ros2 launch pick_place_sim execute_pick_place.launch.py auto_execute:=false
# 纯接触抓取试验；接触验收仍见指南：
ros2 launch pick_place_sim physical_pick_place.launch.py auto_execute:=false
# RGB-D 传感器试验：
ros2 launch pick_place_sim rgbd_pick_place.launch.py auto_execute:=false
# 腕部 ArUco 视觉定位；观察动作仍会自动运行，抓放手动触发：
ros2 launch pick_place_sim vision_pick_place.launch.py auto_execute:=false
# U 形工作台：
ros2 launch pick_place_sim workbench_pick_place.launch.py auto_execute:=false
# 固定薄墙 / 随机障碍 / 静态 Gazebo 真值（任选一条）：
ros2 launch pick_place_sim obstacle_pick_place.launch.py benchmark:=single_wall auto_execute:=false
ros2 launch pick_place_sim obstacle_pick_place.launch.py obstacle_mode:=random obstacle_seed:=42 auto_execute:=false
ros2 launch pick_place_sim obstacle_pick_place.launch.py obstacle_source:=gazebo_truth auto_execute:=false
```

无图形环境在所选 launch 后加 `headless:=true use_rviz:=false`。外部五路 RGB-D、多候选阶段 6 的完整生成配置与启动命令在[指南](docs/guide/RM65B仿真工作流与接口说明.md#扩展方案阶段-6rgb-d-多候选固定附着任务)。

## 3. 发布目标与执行

RViz：`Panels → Add New Panel → rm65b_operator_panel/OperatorPanel`（默认配置自动加载）。拖动原生目标球，或修改面板 TCP/关节并点击应用；先原生 **Plan**，再原生 **Execute**。面板中的 Pick & Place 按钮走独立任务服务。面板限速只约束此 RViz 的原生规划，其他节点的限速须分别配置。

调速：编辑关节“请求上限”、TCP 上限（0—2 m/s；0 显示“不限”）及“规划倍率”（大于 0、至多 1），点击 **应用限速 → Plan → Execute**。生效值受所有约束共同限制；默认 TCP 0.1 m/s 不是输入上限。

```bash
# 新接口：只更新 RViz 目标、进行 IK，不发起运动。需 RViz 面板运行。
# 最稳妥的起点：把当前位姿复制为目标，随后在面板小幅修改。
python3 scripts/publish_current_panel_goal.py
# 手工发布时替换为实际可达的 base_link 位姿（m、单位四元数）：
# ros2 topic pub --once /manipulation/rm65b/set_goal_pose geometry_msgs/msg/PoseStamped \
#   '{header: {frame_id: base_link}, pose: {position: {x: X, y: Y, z: Z}, orientation: {x: QX, y: QY, z: QZ, w: QW}}}'

# 旧接口：收到目标立即规划并执行！只在仿真独立测试时使用。
ros2 run manipulation_demo interactive_target --ros-args -p use_sim_time:=true
# 输入 x y z roll pitch yaw，单位 m/rad。其发布到 /manipulation/rm65b/target_pose。

# 完整抓放的独立入口；先看 plan_ready，再明确执行：
ros2 topic echo /pick_place/plan_ready --once --qos-durability transient_local
ros2 service call /pick_place/execute std_srvs/srv/Trigger '{}'
ros2 service call /pick_place/cancel std_srvs/srv/Trigger '{}'
# 仅支持 reset 的仿真入口，重置不会使旧候选重新有效：
ros2 service call /pick_place/reset std_srvs/srv/Trigger '{}'
```

## 4. 总览、类型、发布者与连接

```bash
ros2 node list
ros2 node info /move_group
ros2 node info /rm65b_operator_panel
ros2 topic list -t
ros2 service list -t
ros2 action list -t
ros2 control list_controllers
ros2 control list_hardware_interfaces
# 下列 TOPIC 替换为任意话题，查看真实类型、发布者、订阅者和 QoS：
ros2 topic info -v /joint_states
ros2 interface show geometry_msgs/msg/PoseStamped
rqt_graph
```

## 5. RGB-D、识别与 TF

```bash
# 图像内容：
ros2 run rqt_image_view rqt_image_view /camera/camera/color/image_raw
ros2 run rqt_image_view rqt_image_view /camera/camera/depth/image_rect_raw
# 图像头/帧率；避免把整幅像素数组刷进终端：
ros2 topic echo /camera/camera/color/image_raw --once --field header --qos-reliability best_effort
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/depth/image_rect_raw
ros2 topic echo /camera/camera/color/camera_info --once --qos-reliability best_effort
ros2 topic echo /camera/camera/depth/camera_info --once --qos-reliability best_effort
ros2 topic echo /camera/camera/depth/color/points --once --field header --qos-reliability best_effort
# 物体/放置输入；已知场景输入不是视觉识别结果：
ros2 topic echo /object_pose --once --qos-durability transient_local
ros2 topic echo /place_pose --once --qos-durability transient_local
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
ros2 run tf2_ros tf2_echo base_link ctm2f110_tool0
# 裸臂把 ctm2f110_tool0 改成 Link6。
# 外部多相机场景，把 /camera 换成 /stage4、/xleft、/xright、/yfront、/yback：
ros2 topic info -v /xleft/camera/depth/image_rect_raw
```

## 6. 末端输入、IK 输出与实际关节

```bash
# 新面板输入（先于 IK）和已接受的目标（经 IK 或关节编辑）：
ros2 topic echo /manipulation/rm65b/set_goal_pose
ros2 topic echo /manipulation/rm65b/ik_target_pose --qos-durability transient_local
ros2 topic echo /manipulation/rm65b/goal_pose --qos-durability transient_local
ros2 topic echo /manipulation/rm65b/goal_joints --qos-durability transient_local
# 当前 FK、物体坐标、限速与失败原因（需面板运行）：
ros2 topic echo /manipulation/rm65b/current_pose --once --qos-durability transient_local
ros2 topic echo /manipulation/rm65b/object_in_base --once --qos-durability transient_local
ros2 topic echo /manipulation/rm65b/object_in_tcp --once --qos-durability transient_local
ros2 topic echo /manipulation/rm65b/planning_limits --once --qos-durability transient_local
ros2 topic echo /manipulation/rm65b/panel_status --qos-durability transient_local
# 实际反馈来自 Gazebo 控制器，不是 IK 目标：
ros2 topic echo /joint_states --once --qos-reliability best_effort
ros2 topic echo /rm65b_arm_controller/controller_state --once
ros2 topic echo /ctm2f110_hand_controller/controller_state --once
# 旧终端目标：
ros2 topic echo /manipulation/rm65b/target_pose
# 独立 IK/FK 服务定义及实际端点；服务响应不会自动另发成关节命令：
ros2 service type /compute_ik
ros2 interface show moveit_msgs/srv/GetPositionIK
ros2 service type /compute_fk
ros2 interface show moveit_msgs/srv/GetPositionFK
```

`ik_target_pose` 是尝试值；IK 失败时 `goal_pose/goal_joints` 保留上次接受值，不能将它误认为本次 IK 成功。拖动输入也在 MoveIt 的交互标记反馈中；用下面命令找出其当前实际名称：

```bash
ros2 topic list -t | rg 'feedback|interactive_marker'
# ros2 topic echo <上面列出的 /.../feedback> --field pose
```

## 7. 规划器输入输出、轨迹与执行

```bash
# 原生 OMPL 请求（MoveGroup 调试发布；实际端点用 topic list 核对）：
ros2 topic echo /motion_plan_request
# 路径与时间参数化后的关节轨迹：
ros2 topic echo /display_planned_path --once
ros2 topic echo /display_planned_path --once --field trajectory
# MoveGroup 与轨迹执行状态；action 的 goal/result 走服务，不是普通 goal 话题：
ros2 action info /move_action
ros2 action info /execute_trajectory
ros2 topic echo /move_action/_action/status
ros2 topic echo /execute_trajectory/_action/status
# 控制器实际接收的轨迹 action 与期望/实际/误差：
ros2 action info /rm65b_arm_controller/follow_joint_trajectory
ros2 action info /ctm2f110_hand_controller/follow_joint_trajectory
ros2 topic echo /rm65b_arm_controller/controller_state --field desired
ros2 topic echo /rm65b_arm_controller/controller_state --field actual
ros2 topic echo /rm65b_arm_controller/controller_state --field error
# MTC 输入 /object_pose、/place_pose、场景和当前关节；输出：
ros2 topic echo /pick_place/task_description --once --qos-durability transient_local
ros2 topic echo /pick_place/solution --once --qos-durability transient_local
ros2 topic echo /pick_place/planning_metrics --once --qos-durability transient_local
ros2 topic echo /pick_place/plan_ready --once --qos-durability transient_local
ros2 topic echo /pick_place/execution_status --qos-durability transient_local
ros2 topic echo /pick_place/execution_success --qos-durability transient_local
ros2 action info /execute_task_solution
```

若 `/motion_plan_request` 在所用 MoveIt 构建中未启用，不能靠 echo 读取 action 请求内容；核对 `/move_action` 的接口定义与 launch 日志。MTC Cartesian 段不会各自经过 OMPL，该段轨迹在 `/pick_place/solution` 中。

## 8. 场景、障碍、附着与夹爪

```bash
ros2 topic echo /monitored_planning_scene --once
# 获取完整场景，而不是只看到增量 diff：
ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene '{components: {components: 1023}}'
ros2 topic echo /perception/collision_objects --once --qos-durability transient_local
ros2 topic echo /perception/scene_ready --once --qos-durability transient_local
ros2 topic echo /perception/scene_revision --once --qos-durability transient_local
ros2 topic echo /perception/obstacle_status --qos-durability transient_local
ros2 topic echo /perception/obstacles_ready --qos-durability transient_local
# 只在对应 RGB-D / 真值刷新入口使用：
ros2 service call /perception/capture_obstacles std_srvs/srv/Trigger '{}'
ros2 service call /obstacles/refresh_truth std_srvs/srv/Trigger '{}'
# 仿真执行证据；仅对应入口才存在：
ros2 topic echo /pick_place/sim/object_pose
ros2 topic echo /pick_place/sim/attachment_state
ros2 topic echo /pick_place/sim/contact_state
ros2 topic echo /ctm2f110_hand_controller/controller_state --once
ros2 param dump /ctm2f110_hand_controller
ros2 param get /robot_state_publisher robot_description
```

**抓取力：当前仿真是 position 控制，没有可读写的“抓取力 N”接口。** `grip_target` 是夹爪关节位置，`contact_state` 是接触状态；`JointState.effort`（若存在）也不能直接当成指尖夹持力。厂商 SDK 适配器当前同样只接线位置/回读；实机力控制需另行实现和标定。

## 9. 验证与迁移

```bash
bash scripts/verify_realman_gz_runtime.sh
python3 scripts/verify_realman_gripper_gz_runtime.py
python3 scripts/verify_pick_place_scene.py        # 旧单桌
python3 scripts/verify_mtc_stage2.py             # 完整规划
python3 scripts/verify_pick_place_stage3.py      # 固定附着完整抓放
python3 scripts/verify_pick_place_stage4.py      # 纯接触，须执行前启动
python3 scripts/verify_pick_place_stage5.py --wrist-camera
python3 scripts/verify_pick_place_stage6.py      # 视觉定位/场景门槛
python3 scripts/verify_stage1_workbench.py        # U 台
# 在单桌 known_scene 仿真运行且没有其他 RViz 操作时，探针会小幅执行 joint1：
bash scripts/test_operator_panel.sh
python3 scripts/verify_migration_manifest.py
bash scripts/verify_environment.sh
```

[实机准备与可迁移清点](docs/build-reports/2026-10-27-RM65B操作面板工作流与迁移清点.md) · [原扩展方案与阶段验收](docs/build-reports/2026-10-22-RM65B带障碍物视觉抓取规划扩展方案.md) · [现场验证与标定计划](docs/build-reports/2026-10-20-RM65B实机验证与标定规划.md)

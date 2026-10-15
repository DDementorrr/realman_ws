# RM65B Pick & Place 长时目标任务分类

## 目的与边界

本文是长时目标任务的分类总览，不替代启动指南、实验记录或构建报告。它把“感知目标、生成抓放任务、执行动作、维护仿真状态、验证结果”拆成可独立验收的层，便于后续将 Gazebo 相机和 RM65B 仿真控制器替换为真实设备，而不改动上层任务接口。

工程边界保持不变：Ubuntu 22.04、ROS 2 Humble、Gazebo Harmonic、MoveIt 2；当前仓库只验证仿真，不连接 RM65B 实机，也不把 RS485/Modbus 驱动纳入长时任务。

## 分类模型

| 类别 | 职责 | 当前实现 | 稳定接口 |
|---|---|---|---|
| 场景与仿真 | 提供工作台、物体、放置区、机器人、夹爪和物理反馈 | `robot_gazebo` 的 Pick & Place world；阶段 3 使用固定附着，阶段 4 使用接触抓取 | Gazebo world、`/joint_states`、`/pick_place/sim/contact_state` |
| 目标提供 | 提供目标物体和放置位姿 | `pick_place_demo/known_scene` 发布已知位姿；`aruco_object_pose` 发布视觉位姿 | `/object_pose`、`/place_pose`，均为 `PoseStamped` |
| 任务规划 | 生成可执行的 MTC 抓放任务 | `pick_place_mtc/plan_pick_place` | `/pick_place/task_description`、`/pick_place/solution`、`/pick_place/plan_ready` |
| 运动规划 | 为自由空间和接近/抬升/放置/撤退阶段提供规划 | OMPL 用于自由空间；Cartesian/Pilz LIN 优先用于笛卡尔阶段 | MoveIt PlanningScene、轨迹执行接口 |
| 执行与状态 | 调用轨迹、夹爪动作并确认动作完成 | `pick_place_sim/execute_pick_place.py`；Gazebo 控制器反馈为执行状态来源 | `/pick_place/execute`、`/pick_place/reset`、`/joint_states` |
| 场景一致性 | 同步 Gazebo 物体状态和 MoveIt 附着/分离状态 | `attachment_transport`；视觉节点用 `CollisionObject.ADD` 更新 PlanningScene | attach/detach 事件、PlanningScene collision object |
| 验收与回归 | 检查每个阶段的启动、话题、规划和重复执行 | `scripts/verify_pick_place_stage*.py` | 阶段检查脚本输出和运行日志 |

层之间只通过上述接口连接。视觉层可以被真实 RealSense 节点替换，任务规划层仍只消费 `/object_pose` 和 `/place_pose`；Gazebo 物理层也可以被真实执行器替换，MTC 阶段接口不变。

## 阶段分类与门槛

| 阶段 | 任务类别 | 启动入口 | 进入条件 | 通过门槛 |
|---|---|---|---|---|
| 1 | 已知位姿基础场景 | `pick_place_demo known_scene.launch.py` | RM65B、夹爪和现有 MoveIt 配置可启动 | 工作台、物体、放置区存在；已知 `/object_pose` 和 `/place_pose` 可被规划节点读取 |
| 2 | MTC 抓取放置规划 | `pick_place_mtc plan_pick_place.launch.py` | 阶段 1 场景可复现 | 生成 Home → Pre-Grasp → Approach → Close → Attach → Lift → Transport → Place → Open → Detach → Retreat 任务；自由空间使用 OMPL，笛卡尔段使用 CartesianPath，若部署了 Pilz 则可替换为 LIN |
| 3 | 固定附着状态管理 | `pick_place_sim execute_pick_place.launch.py` | 阶段 2 规划成功 | Gazebo 固定/可拆附着与 PlanningScene attach/detach 同步；执行后可 reset 并重复执行 |
| 4 | 真实接触抓取 | `pick_place_sim physical_pick_place.launch.py` | 阶段 3 接口和复位稳定 | 不依赖强制固定附着；搬运时由夹爪碰撞、摩擦、质量和接触维持物体；连续独立循环结果一致 |
| 5 | RGB-D 仿真感知输入 | `pick_place_sim rgbd_pick_place.launch.py` | 阶段 4 物理场景可启动 | Gazebo Harmonic 原生 RGB、Depth、CameraInfo 和 PointCloud2 输出，并提供接近 RealSense 的重映射 |
| 6 | 视觉定位驱动任务 | `pick_place_sim vision_pick_place.launch.py` | 阶段 5 话题稳定；标记可见 | AprilTag/ArUco 位姿统一转换到 `base_link` 下的 `/object_pose`；PlanningScene 动态更新；MTC 可由视觉位姿驱动完整循环 |

阶段必须按顺序验收。阶段 5 的相机接口和阶段 6 的视觉定位可以单独运行，不应把相机检测代码直接耦合到 MTC 节点中。

## 接口分类

### 输入与场景

- `/object_pose`：目标物体位姿，`geometry_msgs/msg/PoseStamped`。已知场景和视觉节点都实现该接口；视觉版本发布到 `base_link`。
- `/place_pose`：放置位姿，`geometry_msgs/msg/PoseStamped`。当前由场景配置提供。
- `/pick_place/sim/contact_state`：仿真接触观测，用于判断夹爪是否仍与目标物体接触。
- `/pick_place/sim/attachment_state`：固定/可拆附着模式的 Gazebo 状态；真实接触模式不应把它当作抓取成功的唯一证据。

### 任务与执行

- `/pick_place/task_description`：MTC 任务描述，供调试和记录使用。
- `/pick_place/solution`：MTC 规划解。
- `/pick_place/plan_ready`：规划就绪信号；视觉节点可在规划就绪后冻结目标更新，避免执行中目标跳变。
- `/pick_place/execute`、`/pick_place/reset`：执行一轮和复位一轮任务的服务入口。

### RGB-D 与兼容层

Gazebo 原生桥接话题为：

| 语义 | Gazebo 适配话题 | RealSense 兼容话题 |
|---|---|---|
| 彩色图像 | `/pick_place/rgbd/image` | `/camera/camera/color/image_raw` |
| 深度图像 | `/pick_place/rgbd/depth_image` | `/camera/camera/depth/image_rect_raw` |
| 彩色相机内参 | `/pick_place/rgbd/camera_info` | `/camera/camera/color/camera_info` |
| 点云 | `/pick_place/rgbd/points` | `/camera/camera/depth/color/points` |

视觉节点只依赖兼容话题和 TF，不依赖 Gazebo 消息类型；因此真实相机替换时只需保持话题、帧名和消息语义。

## 长时运行状态与失败分类

一轮任务按以下状态记录：`SCENE_READY → POSE_READY → PLAN_READY → EXECUTING → ATTACHED → PLACED → HOME → RESET`。任何状态失败都应保留最后一份日志和对应阶段检查结果，再从 `RESET` 重新开始；不要在执行中重新发布目标位姿或重启单个控制器。

| 失败类别 | 典型证据 | 处理边界 |
|---|---|---|
| 感知失败 | `/object_pose` 超时、TF 无法变换、标记不可见 | 只重试感知和场景更新，不改 MTC 任务定义 |
| 规划失败 | `plan_ready=false`、碰撞或 IK 错误 | 检查目标姿态、PlanningScene 和规划参数；保留场景层接口 |
| 执行失败 | 轨迹 action 超时、`/joint_states` 不跟随 | 检查 Gazebo 控制器和反馈；不能用 RViz 动画代替执行证据 |
| 抓取状态失败 | 接触丢失、附着状态与 PlanningScene 不一致 | 由仿真状态适配层处理 attach/detach；不在视觉节点中修补 |
| 放置/复位失败 | 物体未落入放置区、Home 未到达 | 记录物体最终位姿和关节反馈，调用 reset 后再进行下一轮 |

## 替换矩阵与验收记录

| 当前仿真模块 | 未来真实模块 | 上层保持不变的内容 |
|---|---|---|
| Gazebo RGB-D + bridge | RealSense ROS 2 driver | `/object_pose`、`/place_pose` 和 MTC 节点 |
| `known_scene` | 视觉定位或外部工件跟踪 | PoseStamped 语义和 `base_link` 约定 |
| `attachment_transport` / Gazebo contact | 真实夹爪和机械臂反馈 | attach/detach 事件和任务阶段顺序 |
| Gazebo `FollowJointTrajectory` 控制器 | 经审核的 RM65B/夹爪执行器 | 轨迹目标、关节名和执行状态判定 |

每次长时回归至少保存：启动命令、提交版本、阶段脚本输出、`/object_pose` 与 `/place_pose`、规划结果、关键 `/joint_states` 片段、物体最终位姿及失败分类。构建或环境变更后仍需执行仓库约定的迁移清单和环境检查。

## 相关文档

- [工作空间架构](工作空间架构.md)：源码、上游和运行链边界。
- [RM65B 与知行 CTM2F110 夹爪联合仿真](瑞尔曼RM65B与知行夹爪联合仿真.md)：机器人和夹爪仿真启动说明。
- [迁移清单](../迁移记录/迁移清单.md)：目标环境合同、排除项和完成条件。
- 根目录 [README Pick & Place 章节](../../README.md)：逐阶段启动命令和当前验证记录。

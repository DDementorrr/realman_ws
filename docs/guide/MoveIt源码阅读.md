# 从 FR3 请求追踪到 MoveIt 与 OMPL 算法

## 1. 本轮学习方式

以“原理 → 项目配置 → 成熟实现 → 可观察结果”为每次学习的单位。
教学 core 负责用少量代码呈现一个算法思想；成熟源码解释这个思想如何处理真实工程的状态、
回调、约束、容差和失败。下面链接均指向本地固定版本文件，无需从 MoveIt main 猜测 API。

## 2. 当前打开的 launch 到底做什么

[franka_moveit_gz.launch.py](../../manipulation_ws/src/manipulation_bringup/launch/franka_moveit_gz.launch.py)
选择机器人描述、SRDF、kinematics 和 controller 配置，再配置 OMPL pipeline。
它负责组织组件和启动顺序；规划循环在 C++ 插件中执行。

从 [fr3_kinematics.yaml](../../manipulation_ws/src/robot_franka_moveit_config/config/fr3_kinematics.yaml)
看到 `kdl_kinematics_plugin/KDLKinematicsPlugin`，应跳到 KDL 插件。
从 [ompl_planning.yaml](../../manipulation_ws/src/robot_franka_moveit_config/config/ompl_planning.yaml)
看到 `RRTConnectkConfigDefault` 对应 `geometric::RRTConnect`，再跳到 OMPL 同名类。
配置名不是算法代码，而是选择实现及其参数的入口。

## 3. 按功能查源码

| 板块 | 本地成熟实现 | 阅读重点 |
|---|---|---|
| 用户请求 | [MoveGroupInterface](../../manipulation_ws/upstream/moveit2/moveit_ros/planning_interface/move_group_interface/src/move_group_interface.cpp) | 目标如何变成请求，plan 与 execute 如何分开 |
| 服务端规划/执行 | [Move action capability](../../manipulation_ws/upstream/moveit2/moveit_ros/move_group/src/default_capabilities/move_action_capability.cpp) | 只规划与规划执行分支、结果码 |
| 状态与 FK | [RobotState](../../manipulation_ws/upstream/moveit2/moveit_core/robot_state/src/robot_state.cpp) | 关节状态更新、变换缓存、Jacobian |
| 场景与碰撞 | [PlanningScene](../../manipulation_ws/upstream/moveit2/moveit_core/planning_scene/src/planning_scene.cpp) | 世界障碍、自碰撞、允许碰撞矩阵 |
| IK | [KDL 插件](../../manipulation_ws/upstream/moveit2/moveit_kinematics/kdl_kinematics_plugin/src/kdl_kinematics_plugin.cpp) | searchPositionIK、CartToJnt、种子、重试、限位 |
| IK 数值内核 | [Mimic SVD](../../manipulation_ws/upstream/moveit2/moveit_kinematics/kdl_kinematics_plugin/src/chainiksolver_vel_mimic_svd.cpp) | Jacobian 降维、权重和 SVD 求解；不能简单等同教学 DLS |
| 规划流水线 | [PlanningPipeline](../../manipulation_ws/upstream/moveit2/moveit_ros/planning/planning_pipeline/src/planning_pipeline.cpp) | 请求适配器、插件调用、响应适配器 |
| 插件入口 | [OMPLPlannerManager](../../manipulation_ws/upstream/moveit2/moveit_planners/ompl/ompl_interface/src/ompl_planner_manager.cpp) | MoveIt PlannerManager 接口如何实现 |
| 算法分配 | [PlanningContextManager](../../manipulation_ws/upstream/moveit2/moveit_planners/ompl/ompl_interface/src/planning_context_manager.cpp) | registerDefaultPlanners 与请求中的 planner_id |
| 上下文执行 | [ModelBasedPlanningContext](../../manipulation_ws/upstream/moveit2/moveit_planners/ompl/ompl_interface/src/model_based_planning_context.cpp) | 状态空间、目标采样、超时、solve、简化和结果转换 |
| 碰撞回调 | [StateValidityChecker](../../manipulation_ws/upstream/moveit2/moveit_planners/ompl/ompl_interface/src/detail/state_validity_checker.cpp) | OMPL state 如何映射回机器人状态及场景约束 |
| 时间参数化 | [TOTG](../../manipulation_ws/upstream/moveit2/moveit_core/trajectory_processing/src/time_optimal_trajectory_generation.cpp) | 路径怎样获得满足限制的时间、速度和加速度 |
| 轨迹执行 | [TrajectoryExecutionManager](../../manipulation_ws/upstream/moveit2/moveit_ros/planning/trajectory_execution_manager/src/trajectory_execution_manager.cpp) | 控制器选择、关节拆分、执行监控与失败 |

## 4. 第一堂源码课：RRT-Connect 的一次扩展

先读 [OMPL RRTConnect.cpp](../../manipulation_ws/upstream/ompl/src/ompl/geometric/planners/rrt/src/RRTConnect.cpp)
的 `growTree()`，再读 `solve()`。不要从头逐个 include 开始。

1. `tree->nearest` 找当前树中离随机目标最近的节点。距离由状态空间定义。
2. 如果距离超过 `maxDistance_`，通过 `interpolate` 只前进允许的步长。
3. 检查插值是否真的前进；某些约束空间插值会停在原地。
4. `si_->checkMotion` 判断这一段是否有效。规划器通过回调访问有效性，不在树代码里读取机器人网格。
5. 无效时返回 `TRAPPED`；有效时保存新节点和父指针，返回前进或到达状态。
6. `solve()` 调用上述函数扩展一棵树，再让另一棵树反复连接，最终沿父指针取回路径。

将它与 [教学 rrt_connect.cpp](../../manipulation_ws/src/manipulation_core/src/rrt_connect.cpp)
逐项对照：最近邻、步长、插值、边验证、连接、路径回溯六个思想一致；成熟实现额外处理
空间语义、方向、内存、终止条件和近似解等问题。

继续读 [DiscreteMotionValidator](../../manipulation_ws/upstream/ompl/src/ompl/base/src/DiscreteMotionValidator.cpp)，
理解“两个端点无碰撞”和“中间运动有效”的区别。该离散验证器只是一个实现，具体运行上下文
可能安装不同 motion validator，不能把所有规划都解释成固定步长离散检查。

## 5. 第二堂源码课：相同接口里的不同策略

读 [RRTstar.cpp](../../manipulation_ws/upstream/ompl/src/ompl/geometric/planners/rrt/src/RRTstar.cpp)，
寻找累计代价、候选父节点和重连：它在第一条可行路径后继续改善代价。
读 [PRM.cpp](../../manipulation_ws/upstream/ompl/src/ompl/geometric/planners/prm/src/PRM.cpp)，
寻找采样节点、建边、连通性和查询：理解为何可复用路网适合重复查询。

阅读过程中画出节点/边如何变化，记录一次扩展的输入输出；先解释代码能改变什么，再进行性能比较。
源码读懂不等于某一算法在当前 FR3 场景必然最快，比较方法仍遵循既有 benchmark 指南。

## 6. 第三堂源码课：数值 IK 与轨迹控制

IK 的基本循环是“当前关节 → FK → 位姿误差 → Jacobian → 关节增量 → 重算误差”。
对照 KDL 插件的 `CartToJnt` 与 SVD 内核，观察超时、停滞、关节权重和 mimic joint 的处理。
教学 DLS 解释为什么要稳定地反求关节增量；不要把不同求解器都称为同一个 DLS 实现。

随后从本项目 `response_adapters` 中的 `AddTimeOptimalParameterization` 跟进
[时间参数化适配器](../../manipulation_ws/upstream/moveit2/moveit_ros/planning/planning_response_adapter_plugins/src/add_time_optimal_parameterization.cpp)。
路径有形状但没有运动速度；时间参数化产生轨迹，执行管理器再按关节集合发送给控制器。
MoveIt 执行管理器负责发送和监控，电机/仿真闭环在控制器内运行。

## 7. 一周阅读安排与验收题

| 次序 | 要完成的阅读 | 自己应能回答的问题 |
|---|---|---|
| 1 | launch、kinematics、OMPL YAML 与上表 | 哪个配置选择哪个类？ |
| 2 | 教学 RRT-Connect 与 OMPL growTree/solve | 一个随机样本怎样改变两棵树？ |
| 3 | StateValidityChecker 与 PlanningScene | 碰撞检查在哪里发生，谁提供障碍？ |
| 4 | KDL 插件与 SVD | 数值求解为何可能停滞，种子怎样影响解？ |
| 5 | RRT* 和 PRM | 树优化与路网复用各自维护什么状态？ |
| 6 | TOTG 与执行管理 | 为什么 Plan 成功但 Execute 仍可能失败？ |
| 7 | 从一个 FR3 请求画完整调用链 | 请求参数、机器人模型和反馈状态分别进入哪里？ |

本轮没有启动新的仿真实验。这些是后续练习，不是已完成的运行验收。
本地源码处于阅读模式；断点命中该源码需要另行编译匹配的调试产物，见
[工作空间架构](工作空间架构.md)。

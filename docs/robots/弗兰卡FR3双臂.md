# 固定式 Franka FR3 Duo：双链模型、运动学和动力学

## 1. 官方模型边界

模型来自 `franka_description` 2.9.0 的 `robots/fr3_duo` 和
`accessories/fr3_duo_mount_v0_3`。固定支架 Gazebo bringup 由 Franka Jazzy 文档明确提供；
上游完整 MoveIt config 当前主要面向 Mobile FR3 Duo。本工程为固定支架建立独立 SRDF/
controller adapter，并未复制移动底盘假设。

来源：

- https://github.com/frankarobotics/franka_description
- https://frankarobotics.github.io/docs/doc/franka_ros2_jazzy/franka_gazebo/franka_gazebo_bringup/doc/index.html
- https://frankarobotics.github.io/docs/doc/franka_ros2_jazzy/franka_mobile_fr3_duo_moveit_config/doc/index.html

## 2. 支架到左右基座

`fr3_duo_mount_v0_3/kinematics.yaml`：

| frame | xyz m | rpy rad |
|---|---|---|
| mounting point offset | -0.037, 0, -0.05 | 0, 0, 0 |
| left mount | 0, 0.05018, 0 | -0.89334809, -0.17456074, -0.46334506 |
| right mount | 0, -0.05018, 0 | 0.89334809, -0.17456074, 0.46334506 |

左右单臂各自使用 [FR3 Craig-DH 表](弗兰卡FR3.md)。因此：

$$
\begin{aligned}
{}^{\mathrm{base}}\mathbf{T}_{\mathrm{left\_tcp}}
&={}^{\mathrm{base}}\mathbf{T}_{\mathrm{left0}}
  \operatorname{FK}_{\mathrm{FR3}}(\mathbf{q}_L)
  \mathbf{T}_{\mathrm{tool},L},\\
{}^{\mathrm{base}}\mathbf{T}_{\mathrm{right\_tcp}}
&={}^{\mathrm{base}}\mathbf{T}_{\mathrm{right0}}
  \operatorname{FK}_{\mathrm{FR3}}(\mathbf{q}_R)
  \mathbf{T}_{\mathrm{tool},R}.
\end{aligned}
$$

$L$、$R$ 分别表示左臂和右臂。两式先从公共 base 到各自安装座，再经过单臂正运动学和工具
偏置，最终把两个 TCP 表达到同一坐标系。

这比创建一张 14 行 DH 表更正确：两个 arm 是从支架分叉的两条串联链，并非一条 14 轴
串联链。

## 3. 双臂正运动学和 Jacobian

堆叠两个独立末端 twist：

$$
\begin{bmatrix}
\mathbf{V}_L\\
\mathbf{V}_R
\end{bmatrix}
=
\begin{bmatrix}
\mathbf{J}_L&\mathbf{0}\\
\mathbf{0}&\mathbf{J}_R
\end{bmatrix}
\begin{bmatrix}
\dot{\mathbf{q}}_L\\
\dot{\mathbf{q}}_R
\end{bmatrix}
$$

块对角结构表示：忽略接触约束时，左臂关节速度只直接影响左末端，右臂同理。

若任务是相对位姿，需把 Jacobian 表达到同一 frame，并构造 $\mathbf{J}_{\mathrm{rel}}$；若还要求物体整体运动，
可以同时定义 object twist 和 internal/null-space motion。

## 4. 双臂逆运动学

两个独立 6D pose target 对 14 个关节留下通常 2 维冗余。当前策略：

1. `left_arm`、`right_arm` 各用 KDL 从当前状态附近求候选；
2. 将两个目标同时作为 `both_arms` goal constraint；
3. 14 维规划全过程检查跨臂碰撞；
4. 一次 RobotTrajectory 拆到两控制器执行。

若抓持同一刚体，终点双 IK 不够；必须满足全路径

$$
\left({}^{\mathrm{base}}\mathbf{T}_{\mathrm{left\_tcp}}\right)^{-1}
{}^{\mathrm{base}}\mathbf{T}_{\mathrm{right\_tcp}}
=\mathbf{T}_{\mathrm{relative}}^{\star}
$$

也就是左右夹具间的相对位姿在整条路径上保持目标常量。这是约束流形规划问题，当前里程碑尚未实现。

## 5. 质量和重力

每条臂的 link mass/COM/inertia 与单 FR3 相同。支架数据：质量 `4.093846 kg`，COM
`(-0.011817,-0.00005,0.044431) m`。支架固定时其重力由地面承受，不进入 14 个 arm joint
的关节补偿项。

忽略两臂物理接触时：

$$
\mathbf{g}_{\mathrm{duo}}(\mathbf{q}_L,\mathbf{q}_R)
=
\begin{bmatrix}
\mathbf{g}_L(\mathbf{q}_L)\\
\mathbf{g}_R(\mathbf{q}_R)
\end{bmatrix},
\qquad
\mathbf{M}_{\mathrm{duo}}
=
\operatorname{blockdiag}(\mathbf{M}_L,\mathbf{M}_R)
$$

即两条独立开链的重力向量直接拼接，惯性矩阵按左右臂组成块对角矩阵。

共同物体会引入运动约束和接触力分配；虽然开链刚体质量矩阵仍可从各实体累加，控制问题
不再是两个独立补偿向量。设物体所受目标合力/力矩为 $\mathbf{W}$，抓取矩阵为 $\mathbf{G}$，
左右夹持 wrench 分别为 $\mathbf{w}_L,\mathbf{w}_R$，则需满足：

$$
\mathbf{G}
\begin{bmatrix}\mathbf{w}_L\\\mathbf{w}_R\end{bmatrix}
=\mathbf{W}
$$

在满足物体受力的基础上，可用零空间分配内力，再把每侧夹持 wrench 映射为关节力矩：

$$
\boldsymbol{\tau}_i=\mathbf{J}_i^{\mathsf{T}}\mathbf{w}_i,
\qquad i\in\{L,R\}.
$$

## 6. 控制合同

- 本工程 Gazebo：左右 position JointTrajectoryController，证据等级 D；
- 官方 FR3 Duo 实机 bringup：公开文档目前说明 torque/effort command interface；
- 因此 Gazebo 的 position YAML 不得直接用于实机。

仿真已验收：左右独立执行和 `both_arms` 同次规划/双控制器协调执行均返回成功。闭链物体
操作、真实时间同步和安全认证属于下一阶段。

# PR2 双臂移动机器人资源

## 1. 模型身份和 Gazebo 兼容性

当前资源是 MoveIt tests 中从早期 `pr2_description` 展开的 `robot.xml`。PR2 官方手册称
它为 CAD approximated kinematic and inertial description。上游文件仍包含旧 Gazebo Classic
的 `libgazebo_ros_controller_manager.so` 等 XML/plugin，不能**直接**作为 ROS 2 Jazzy +
Gazebo Harmonic 后端；本工程已用 wrapper/preparer 去除旧插件和 transmission，注入独立的
Harmonic `gz_ros2_control` 描述。

来源：

- https://github.com/PR2/pr2_common
- https://github.com/PR2/pr2_docs/blob/master/pr2_manual/3_pr2_hardware.tex
- https://github.com/moveit/moveit_resources

当前已有 `pr2_planning.urdf.xacro` 与 `pr2_gz.urdf.xacro` 两份描述：前者发布到
`/robot_description`，后者发布到 `/robot_description_gazebo`。启动遵循 paused、inactive
load、unpause、activate 门控；2026-09-04 运行验收中四个 controller 均 active、mesh 资源可
解析。该适配仍只覆盖左右臂、头/升降轴，底盘、传感器、夹爪和 MoveIt 全身执行未现代化。

## 2. 单臂链和等价参数

以左臂 `torso_lift_link -> l_gripper_palm_link` 为例，7 个主动关节轴序列：

| active joint | origin xyz m | axis |
|---|---|---|
| shoulder pan | 0, 0.188, 0 | z |
| shoulder lift | 0.1, 0, 0 | y |
| upper arm roll | 0,0,0 | x |
| elbow flex | 0.4,0,0 | y |
| forearm roll | 0,0,0 | x |
| wrist flex | 0.321,0,0 | y |
| wrist roll | 0,0,0 | x |

右臂肩部 y 偏置镜像。由于轴在一个分支树中并包含 fixed link，保留 URDF 乘积比伪造单一
DH 表更可审计：

$$
\begin{aligned}
\mathbf{T}={}&
\operatorname{Ty}(0.188)\operatorname{Rz}(q_1)
\operatorname{Tx}(0.1)\operatorname{Ry}(q_2)
\operatorname{Rx}(q_3)\\
&\operatorname{Tx}(0.4)\operatorname{Ry}(q_4)
\operatorname{Rx}(q_5)
\operatorname{Tx}(0.321)\operatorname{Ry}(q_6)
\operatorname{Rx}(q_7)\mathbf{T}_{\mathrm{palm}}.
\end{aligned}
$$

各项严格按 URDF parent → child 顺序相乘；平移项来自 fixed/active joint origin，旋转项来自
对应主动关节。

若建立 DH 表，必须明确把 fixed upper-arm/forearm link 合并到哪一行，并用提取脚本与原
URDF 随机回归。

## 3. 正运动学和全身状态

单臂 FK 还要左乘可移动底盘和 torso：

$$
{}^{\mathrm{map}}\mathbf{T}_{\mathrm{palm}}
=
{}^{\mathrm{map}}\mathbf{T}_{\mathrm{base}}(x,y,\psi)
{}^{\mathrm{base}}\mathbf{T}_{\mathrm{torso}}(q_{\mathrm{torso}})
{}^{\mathrm{torso}}\mathbf{T}_{\mathrm{palm}}(\mathbf{q}_{\mathrm{arm}})
$$

$\psi$ 是底盘偏航角，$q_{\mathrm{torso}}$ 是升降关节，$\mathbf{q}_{\mathrm{arm}}$ 是七个手臂
关节。只有三部分处于同一状态，才能得到手掌相对地图的完整位姿。

只规划 arm 时底盘/torso 固定；full-body planning 则把移动 base、torso、arm 放进复合状态。
这也是 PR2 不能只用一张 7 行 DH 表描述整机的原因。

## 4. IK 解析

每条 PR2 arm 是 7R 冗余链，6D 目标通常有一维连续自由度。可选择 elbow/arm angle 后做
半解析分支，也可用数值 IK。对全身目标，自由度更多，通常通过分层优化或约束 IK 分配给
base、torso 和 arm。当前模型未随附可声明为 OEM 认证的闭式 IK；正确验收同样是 FK 回代、
限位、碰撞和连续性。

## 5. 质量参数

历史 URDF 给出左臂代表值：

| link | mass kg | COM xyz m |
|---|---:|---|
| shoulder_pan | 25.799322 | -0.001201, 0.024513, -0.098231 |
| shoulder_lift | 2.74988 | 0.02195, -0.02664, -0.03127 |
| upper_arm_roll placeholder | 0.1 | 0,0,0 |
| upper_arm | 6.01769 | 0.21405, 0.01658, -0.00057 |
| elbow_flex | 1.90327 | 0.01014, 0.00032, -0.01211 |
| forearm_roll placeholder | 0.1 | 0,0,0 |
| forearm | 2.57968 | 0.18791, -0.00017, -0.00912 |
| wrist_flex | 0.61402 | -0.00157, 0, -0.00075 |
| wrist_roll placeholder | 0.1 | 0,0,0 |
| gripper palm | 0.58007 | 0.06623, 0.00053, -0.00119 |

表中多个 `0.1 kg` 明显是占位刚体；官方手册也把整个模型定性为 CAD approximation，不能
当高精度辨识数据。

## 6. 重力补偿

对 arm active joints 仍按：

$$
\boldsymbol{\tau}_{\mathrm{comp}}
=
-\sum_k
\mathbf{J}_{v,\mathrm{com},k}^{\mathsf{T}}
m_k\mathbf{g}_0
$$

$k$ 遍历具有质量的连杆，包括通过 fixed joint 连接的连杆；它们虽然没有独立自由度，仍会
对上游主动关节产生重力矩。

fixed link 也有质量：它虽没有自己的主动 DOF，重力仍通过所有上游主动关节产生力矩，不能
从求和中删掉。移动底盘姿态和 torso 高度会改变 COM 的世界变换。真实 PR2 wrist 还有
cross-coupled transmission；关节力矩到 actuator effort 的映射不能用单位矩阵代替。

当前数据适合教学交叉验证，不作为真实 PR2 精确补偿验收基准。

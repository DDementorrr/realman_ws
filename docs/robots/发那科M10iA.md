# FANUC M-10iA MoveIt 测试模型

## 1. 来源等级与适用范围

- B：MoveIt 官方 `moveit_resources/fanuc_description`，用于规划测试；
- C：`ros-industrial/fanuc` 的 M-10iA support 是社区支持、ROS 1/Noetic 路线，README 明确
  不是 OEM 支持；
- A：FANUC America M-10iA/12 页面给出 6 轴、12 kg、1420 mm reach 等产品级总规格，但
  未在公开页面给出本项目所需每 link 质量/COM/惯量。

来源：

- https://github.com/moveit/moveit_resources
- https://github.com/ros-industrial/fanuc
- https://www.fanucamerica.com/products/robot/m-10ia-12

所以当前模型可验证“6 轴接口不被硬编码破坏”，不能作为真实 M-10iA 动力学或标定模型。

## 2. URDF 串联链参数

| joint | parent → child 原点位置（m） | 转轴 |
|---|---|---|
| J1 | 0,0,0.450 | z |
| J2 | 0.150,0,0 | +y |
| J3 | 0,0,0.600 | -y |
| J4 | 0,0,0.200 | -x |
| J5 | 0.640,0,0 | -y |
| J6 | 0.100,0,0 | -x |
| tool0 fixed | $0,0,0$；RPY $=(\pi,-\pi/2,0)$ | - |

这是一张 URDF origin/axis 表，不是经 OEM 发布的 DH 表。可以按每行构造：

$$
\mathbf{T}_i(q_i)
=
\operatorname{Trans}(x_i,y_i,z_i)
\operatorname{Rot}(\mathbf{a}_i,q_i)
$$

其中，$q_i$ 是第 $i$ 个关节角，$(x_i,y_i,z_i)$ 是该 joint 的 URDF `origin`，
$\mathbf{a}_i$ 是 `axis` 给出的转轴。整条机械臂变换为：

$$
{}^{\mathrm{base}}\mathbf{T}_{\mathrm{tool0}}
=
\mathbf{T}_1\mathbf{T}_2\cdots\mathbf{T}_6\mathbf{T}_{\mathrm{fixed}}
$$

读法是“从 base 出发，依次经过六个主动关节，最后乘上 tool0 的固定变换”。

若必须转成 DH，要重新选择 z 轴沿每个关节轴、令 x 轴沿相邻 z 轴公法线，并求
$a_i,\alpha_i,d_i,\theta_{\mathrm{offset},i}$；转换后应在大量随机 $\mathbf{q}$ 上与上述
URDF 连乘逐点一致。未完成
这种校验前，不把推导表标成“FANUC 官方 DH”。

## 3. 正运动学

使用一般轴角表示更忠实：

$$
\begin{aligned}
\mathbf{T}(\mathbf{q})={}&
\operatorname{Tz}(0.450)\operatorname{Rz}(q_1)\\
&\operatorname{Tx}(0.150)\operatorname{Ry}(q_2)\\
&\operatorname{Tz}(0.600)\operatorname{Ry}(-q_3)\\
&\operatorname{Tz}(0.200)\operatorname{Rx}(-q_4)\\
&\operatorname{Tx}(0.640)\operatorname{Ry}(-q_5)\\
&\operatorname{Tx}(0.100)\operatorname{Rx}(-q_6)\\
&\operatorname{RPY}(\pi,-\pi/2,0).
\end{aligned}
$$

每一行都对应 URDF 中的一级固定平移和关节旋转；负号来自原模型的关节轴方向，而不是计算时
临时添加的补偿。

矩阵右乘顺序以 URDF parent->child 为准。该表达式可直接手算/实现，无需为了使用 DH 而
引入未经验证的 frame 偏置。

## 4. 解析 IK 的结构性推导

传统 6R 的可用解析路线是腕心分解：

$$
\mathbf{p}_{\mathrm{wc}}
=
\mathbf{p}_{\mathrm{tcp}}
-\mathbf{R}_{\mathrm{tcp}}\mathbf{p}_{6\rightarrow\mathrm{tcp}}
$$

$\mathbf{p}_{\mathrm{wc}}$ 是腕心位置：从 TCP 位置减去末端工具偏置。随后用腕心几何求
$q_1,q_2,q_3$，再计算腕部剩余旋转：

$$
{}^{3}\mathbf{R}_{6}
=
\left({}^{0}\mathbf{R}_{3}(q_1,q_2,q_3)\right)^{\mathsf{T}}
{}^{0}\mathbf{R}_{6}
$$

最后从 ${}^{3}\mathbf{R}_{6}$ 提取 $q_4,q_5,q_6$，并枚举腕部翻转分支。

但是“后三轴是否严格相交、offset 符号、零位和分支公式”必须由经过 OEM/标定验证的几何
确认。当前测试 URDF只足以实现数值 IK；本工程不声称已经得到真实 M-10iA 的 OEM 闭式解。
若未来引入 IKFast，也必须做随机 FK 回代、全部关节范围、奇异点和实机标定验证。

## 5. 质量和重力补偿缺口

当前 Fanuc URDF 所有 link 都没有 `<inertial>`，joint effort 甚至为 0 占位。整机质量
`130 kg` 不能分解出各 link 的 mass/COM。因此：

$$
\boldsymbol{\tau}_{\mathrm{comp}}(\mathbf{q})
=
-\sum_k
\mathbf{J}_{v,\mathrm{com},k}(\mathbf{q})^{\mathsf{T}}
m_k\mathbf{g}_0
$$

这里 $m_k$ 是第 $k$ 个连杆质量，$\mathbf{J}_{v,\mathrm{com},k}$ 是其质心线速度 Jacobian，
$\mathbf{g}_0$ 是重力加速度向量。公式成立，但 $m_k$、$\mathbf{p}_{\mathrm{com},k}$ 未知，不能数值计算。下一步要向 FANUC/系统集成商索取与精确
variant 匹配的 CAD/动力学/ROBOGUIDE 数据或使用厂商控制器内部补偿；不得按均分整机质量
估算并用于实机。

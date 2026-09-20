# Franka Research 3（FR3）参数、运动学与重力补偿

## 1. 来源和版本

- A 级：Franka Control Interface 官方规格
  https://frankarobotics.github.io/docs/robot_specifications.html
- A 级：`frankarobotics/franka_description` tag `2.9.0`，本地 commit
  `7aeeddc449edf8d62b594f9e36a81da53e7796f9`
- 本地原始参数：`manipulation_ws/src/franka_description/robots/fr3/`

用户提供的中文镜像 https://www.tacbot.cn/FCI/index.html 适合辅助阅读，但参数验收以 Franka
域名文档和厂家 GitHub 固定版本为准。

## 2. Craig 修正 DH

约定：

$$
{}^{i-1}\mathbf{T}_{i}
=
\operatorname{RotX}(\alpha_i)
\operatorname{TransX}(a_i)
\operatorname{RotZ}(q_i)
\operatorname{TransZ}(d_i)
$$

左上角 $i-1$ 是父坐标系，右下角 $i$ 是子坐标系；它们是坐标系编号，不表示乘方。

| $i$ | $a_i$（m） | $d_i$（m） | $\alpha_i$ | $\theta_i$ |
|---:|---:|---:|---:|---|
| 1 | 0 | 0.333 | 0 | $q_1$ |
| 2 | 0 | 0 | $-\pi/2$ | $q_2$ |
| 3 | 0 | 0.316 | $\pi/2$ | $q_3$ |
| 4 | 0.0825 | 0 | $\pi/2$ | $q_4$ |
| 5 | -0.0825 | 0.384 | $-\pi/2$ | $q_5$ |
| 6 | 0 | 0 | $\pi/2$ | $q_6$ |
| 7 | 0.088 | 0 | $\pi/2$ | $q_7$ |
| flange | 0 | 0.107 | 0 | 0 |

## 3. 正运动学展开方法

可用 $c_i=\cos(q_i)$、$s_i=\sin(q_i)$ 缩短手算。每行先构造：

$$
\mathbf{A}_i
=
\operatorname{Rx}(\alpha_i)
\operatorname{Tx}(a_i)
\operatorname{Rz}(q_i)
\operatorname{Tz}(d_i)
$$

最终：

$$
\begin{aligned}
{}^{0}\mathbf{T}_{\mathrm{flange}}(\mathbf{q})
&=\mathbf{A}_1(q_1)\mathbf{A}_2(q_2)\cdots
  \mathbf{A}_7(q_7)\mathbf{A}_{\mathrm{flange}},\\
{}^{0}\mathbf{T}_{\mathrm{tcp}}(\mathbf{q})
&={}^{0}\mathbf{T}_{\mathrm{flange}}
  {}^{\mathrm{flange}}\mathbf{T}_{\mathrm{tcp}}.
\end{aligned}
$$

第一行算到机器人法兰，第二行再加入工具偏置；更换工具不需要修改七行 DH 参数。

7 个 4×4 矩阵完全符号展开会产生大量三角项，不利于审计；工程和教材保留连乘形式，逐级
比较 ${}^{0}\mathbf{T}_{i}$。`manipulation_core::DhChain` 选择 `kModifiedCraig` 即可计算。验证必须与
URDF 使用相同 `link0/link8` 和 tool frame。

几何 Jacobian 第 $i$ 列：

$$
\mathbf{J}_i
=
\begin{bmatrix}
\mathbf{z}_i\times(\mathbf{p}_{\mathrm{tcp}}-\mathbf{p}_i)\\
\mathbf{z}_i
\end{bmatrix}
$$

这里 $\mathbf{z}_i,\mathbf{p}_i$ 是 Craig 行中
$\operatorname{Rx}(\alpha_i)\operatorname{Tx}(a_i)$ 后、关节变量作用前的轴和原点。上三行
计算线速度贡献，下三行计算角速度贡献。

## 4. 关节限制

官方 description 2.9.0：

| joint | lower | upper | max velocity (rad/s) | effort (N m) |
|---|---:|---:|---:|---:|
| 1 | -2.9007 | 2.9007 | 2.62 | 87 |
| 2 | -1.8361 | 1.8361 | 2.62 | 87 |
| 3 | -2.9007 | 2.9007 | 2.62 | 87 |
| 4 | -3.0770 | -0.1169 | 2.62 | 87 |
| 5 | -2.8763 | 2.8763 | 5.26 | 12 |
| 6 | 0.4398 | 4.6216 | 4.18 | 12 |
| 7 | -3.0508 | 3.0508 | 5.26 | 12 |

这些是模型限制，不替代 FCI 中位置相关速度/减速、jerk、力矩率和安全条件。

## 5. 逆运动学解析

FR3 是 7R：任务位姿通常为 6 维，所以一般解不是有限组 $\mathbf{q}=f(\mathbf{T})$，而是一族
$\mathbf{q}=f(\mathbf{T},\psi,\mathrm{branch})$；$\psi$ 可选为 arm angle/肘部冗余参数。要构造半解析求解器可：

1. 固定 $\psi$ 或 $q_7$；
2. 用几何关系/多项式求有限分支；
3. FK 回代；
4. 检查上述关节限制；
5. 以 seed 距离、限位裕量和碰撞选解；
6. 沿连续轨迹保持分支连续。

当前工程没有声称提供厂家认证闭式 FR3 IK；`DampedLeastSquaresIk` 使用阻尼最小二乘：

$$
\Delta\mathbf{q}
=
\mathbf{J}^{\mathsf{T}}
\left(\mathbf{J}\mathbf{J}^{\mathsf{T}}+\lambda^2\mathbf{I}\right)^{-1}
\mathbf{e}
$$

该式计算每轮关节修正量；MoveIt 运行路径使用 KDL 插件。验收看 FK 回代误差，
不能要求关节角唯一。

## 6. 质量、COM 和惯量

厂家 description 2.9.0 的 arm link 数据：

| link | mass kg | COM xyz m |
|---|---:|---|
| link0 | 2.3966 | -0.0172, 0.0004, 0.0745 |
| link1 | 2.9274653454 | 0.0000004128, -0.0181251324, -0.0386035970 |
| link2 | 2.9355370338 | 0.0031828864, -0.0743221644, 0.0088146084 |
| link3 | 2.2449013699 | 0.0407015686, -0.0048200565, -0.0289730823 |
| link4 | 2.6155955791 | -0.0459100965, 0.0630492960, -0.0085187868 |
| link5 | 2.3271207594 | -0.0016039605, 0.0292536262, -0.0972965990 |
| link6 | 1.8170376524 | 0.0597131221, -0.0410294666, -0.0101692726 |
| link7 | 0.6271432862 | 0.0045225817, 0.0086261921, -0.0161633251 |

完整惯性张量分量 $I_{xx},I_{xy},\ldots$ 保留在 `inertials.yaml`，避免在手抄表中丢符号或
精度。link0 是固定基座，
纯 arm 关节重力项主要累加 link1..link7；工具和 payload 必须作为额外刚体加入。

## 7. 重力补偿推导

势能：

$$
U(\mathbf{q})
=
-\sum_k m_k\mathbf{g}_0^{\mathsf{T}}
\mathbf{p}_{\mathrm{com},k}(\mathbf{q})
$$

对应的重力广义力和补偿力矩为：

$$
\begin{aligned}
\boldsymbol{\tau}_{\mathrm{gravity}}(\mathbf{q})
&=-\frac{\partial U}{\partial\mathbf{q}}
=\sum_k\mathbf{J}_{v,\mathrm{com},k}(\mathbf{q})^{\mathsf{T}}m_k\mathbf{g}_0,\\
\boldsymbol{\tau}_{\mathrm{comp}}(\mathbf{q})
&=\frac{\partial U}{\partial\mathbf{q}}
=-\sum_k\mathbf{J}_{v,\mathrm{com},k}(\mathbf{q})^{\mathsf{T}}m_k\mathbf{g}_0.
\end{aligned}
$$

$k$ 是连杆编号，$m_k$ 是质量，$\mathbf{p}_{\mathrm{com},k}$ 是质心位置，
$\mathbf{J}_{v,\mathrm{com},k}$ 是质心线速度 Jacobian。

计算过程：用 DH 得到每个 child frame，乘相应 COM 坐标得到世界 COM；对 link k 只累加
其上游关节 $i\le k$。本工程 `GravityCompensation` 正是此式。

重要接口语义：官方 Franka 示例中的零外部力矩 gravity compensation 依赖机器人内部已经
补偿自身重量；不要再把本计算值重复叠加。实机请用 libfranka/官方 model interface 输出
并按官方控制合同执行。

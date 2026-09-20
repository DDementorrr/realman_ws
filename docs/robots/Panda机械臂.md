# Franka Panda（MoveIt resource 模型）

## 1. 数据身份

当前 Panda 来自 MoveIt 官方测试资源 `moveit_resources` 的 `ros2` 分支固定 commit
`c55b102711fc0aebe80c6952d2ce97c38110abba`。它适合 MoveIt 回归和教程，但不是当前 Franka
FR3 产品描述。其 `panda.urdf` 只有视觉/碰撞/关节限制，没有 link inertial。

来源：https://github.com/moveit/moveit_resources

## 2. 修正 DH / URDF 几何

该资源的七轴几何与经典 Panda 表对应：

| $i$ | $a$（m） | $d$（m） | $\alpha$ | $\theta$ |
|---:|---:|---:|---:|---|
| 1 | 0 | 0.333 | 0 | $q_1$ |
| 2 | 0 | 0 | $-\pi/2$ | $q_2$ |
| 3 | 0 | 0.316 | $\pi/2$ | $q_3$ |
| 4 | 0.0825 | 0 | $\pi/2$ | $q_4$ |
| 5 | -0.0825 | 0.384 | $-\pi/2$ | $q_5$ |
| 6 | 0 | 0 | $\pi/2$ | $q_6$ |
| 7 | 0.088 | 0 | $\pi/2$ | $q_7$ |
| link8 | 0 | 0.107 | 0 | 0 |

约定仍是
$\operatorname{Rx}(\alpha)\operatorname{Tx}(a)\operatorname{Rz}(\theta)\operatorname{Tz}(d)$。
不要因为尺寸相同就把 FR3 的关节限制、质量
和控制能力套到 Panda；二者产品参数不同。

## 3. FK 推导

$$
\begin{aligned}
{}^{0}\mathbf{T}_{8}
&=\left(\prod_{i=1}^{7}\mathbf{A}_i(q_i)\right)
  \operatorname{Tz}(0.107),\\
{}^{0}\mathbf{T}_{\mathrm{hand}}
&={}^{0}\mathbf{T}_{8}\operatorname{Rz}(-\pi/4).
\end{aligned}
$$

乘积符号表示按 $i=1$ 到 $7$ 的顺序逐级相乘；第二行再加入 hand 相对 link8 的固定旋转。

若 TCP 在 hand/finger 上，必须继续乘固定变换。当前 Panda Gazebo/MoveIt 验收时 planning
tip 与 controller joint set 分开：hand 是单独 planning/control group。

## 4. IK 解析

Panda 是 7R 冗余结构；给 6D 位姿不存在唯一 7 角解析向量。可以固定 arm angle/某关节后
求分支，也可用数值 Jacobian 法。当前工程 MoveIt demo 使用配置的 KDL 插件，教学核心使用
DLS。应以 FK 回代、限位和碰撞验收。

## 5. 质量参数和重力补偿结论

该 MoveIt resource 的 Panda URDF 没有 `<inertial>`，所以从当前模型无法计算有物理意义的
Panda 重力向量：

缺失量可以写成：

$$
\{m_k,\mathbf{p}_{\mathrm{com},k},\mathbf{I}_k\}
\ \text{缺失}
\quad\Longrightarrow\quad
\mathbf{g}(\mathbf{q}),\mathbf{M}(\mathbf{q})
\ \text{不能通过当前资源可靠计算}
$$

若未来要控制真实 Panda，应另外固定厂家维护且与硬件版本匹配的 description/libfranka
模型，记录 payload，并用质心 Jacobian 重力公式或厂家 model API 交叉校验。不得为了
“文档完整”从 FR3 复制 link mass。

## 6. 当前执行器经验

arm+hand 规划会生成两个 trajectory part。若
`/panda_hand_controller/gripper_cmd` action server 未启动，MoveIt 会取消已发送的 arm part，
造成 plan 成功但 execute ABORTED。当前 Panda Gazebo bringup 已配置 arm 和 parallel gripper
两个真实仿真 action server；运行验收脚本会同时检查它们。

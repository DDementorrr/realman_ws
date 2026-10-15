# RealSense 无标记定位与 Git 实机互通方案

日期：2026-10-15  
适用工程：Ubuntu 22.04、ROS 2 Humble、RM65B + RealSense + MoveIt

## 1. 结论

RealSense 本身提供 RGB、深度、红外、CameraInfo 和点云数据，不会自动给出任意物体的 6D 位姿。无 ArUco 定位必须在相机数据上增加一个感知算法。算法可能依赖：

- 深度几何（平面、法向、聚类、尺寸先验），不需要纹理特征；
- RGB/点云的局部或全局特征（边缘、纹理、关键点、描述子、模型对应关系）；
- 训练好的检测/姿态网络（语义特征由模型学习，而不是人工标签）。

当前最适合实机第一版的是“深度平面分割 + 欧式聚类 + 物体尺寸/方向先验”，或“RGB 检测框 + 对齐深度”。它们比 ArUco 泛化，但仍需要已知类别、工作台几何或尺寸约束。若目标完全未知、遮挡严重且需要稳定 6D 姿态，应采用带 CAD/训练数据的模型匹配或 6D pose 网络，不能把普通深度图直接当作位姿传感器。

## 2. 无 ArUco 算法选型

| 方法 | 是否依赖视觉特征 | 输出与先验 | 适用性与限制 |
|---|---|---|---|
| 深度平面分割 + 聚类 | 不依赖纹理；依赖深度几何、法向和边界 | 桌面上方物体点云、中心和主方向 | 最易落地、可解释；对透明/黑色/反光表面、相邻接触物体和遮挡敏感 |
| RGB-D 背景差分/占据栅格 | 可不依赖纹理；依赖背景模型 | 前景区域、质心、包围盒 | 适合固定工位和单物体；相机或桌面变化会破坏背景模型 |
| RGB 检测器（YOLO 等）+ 深度 | 依赖学习到的 RGB 语义特征 | 类别框、中心深度、粗位置 | 类别泛化好但需要训练/验证；检测框不是 6D 姿态，深度遮挡会造成偏差 |
| 传统关键点/局部特征 + PnP | 依赖纹理/边缘关键点 | 相机坐标下 6D pose | 不需 ArUco，但需纹理稳定、已知 3D 点和相机内参；弱纹理物体失败 |
| CAD 模型 + ICP/NDT | 依赖点云几何特征和初始位姿 | 6D pose | 适合已知刚体；需要 CAD、尺度、较好初值，反光/稀疏深度和对称物体会退化 |
| Point Pair Features / 全局模型匹配 | 依赖点云几何结构 | 6D pose，可弱化初值要求 | 对纹理要求低；计算量、遮挡和对称性处理复杂 |
| 深度学习 6D pose（PoseCNN、FoundationPose 等） | 依赖训练或预训练视觉特征 | 类别级或实例级 6D pose | 泛化潜力最高但部署重量、数据集、GPU、许可证和现场验证成本最高 |
| 视觉伺服（IBVS/PBVS） | 依赖图像/姿态误差特征 | 运动增量，不是独立定位器 | 是闭环控制层，可叠加在上述定位器之后；不能替代可靠的初始检测 |

RealSense SDK/ROS wrapper 负责设备通信、时间戳、内参、深度对齐和点云生成；OpenCV、PCL、深度学习框架或第三方 6D pose 库负责定位算法。`realsense2_camera` 不等于物体识别库。

### 2.1 推荐的实机演进顺序

1. **深度基线**：固定工作台、单个有明显高度差的物体，使用 ROI、桌面平面拟合、去平面、聚类、质心和尺寸过滤，发布 `base_link` 下 `/object_pose`。
2. **RGB-D 类别定位**：增加一个成熟 RGB 检测器，只用检测框裁剪对齐深度；对深度点做中位数/平面拟合，避免单像素噪声。
3. **已知物体 6D**：有 CAD 时使用点云配准或 PnP+关键点，输出姿态置信度和残差；对称物体明确姿态等价类。
4. **复杂场景**：再评估 FoundationPose、PoseCNN 等模型，并在工控机上测量 GPU、延迟、失效率和许可证。

无论选择哪种算法，保持上层契约不变：`PoseStamped.header.frame_id=base_link`，同时发布定位时间戳、有效期、置信度/残差和失败状态。MTC 不应知道目标来自 ArUco、平面分割还是神经网络。

## 3. 当前工程适合提交 Git 的内容

### 3.1 建议提交

- `src/` 下自建 ROS 2 包：感知适配、`/object_pose` 接口、MTC 任务、MoveIt 配置、机器人/夹爪描述和实机 observer 节点。
- `launch/`、参数 YAML、URDF/Xacro、SRDF、控制接口声明、RViz 配置和话题/QoS 契约。
- `scripts/` 中不包含密钥且可复现的检查脚本；实机检查脚本必须默认只读，不得隐式连接或运动。
- `docs/`、`AGENTS.md`、迁移清单、算法来源、许可证和已知限制。
- `upstream/*.repos`、来源说明、版本/哈希清单以及导入脚本。它们描述如何复现上游依赖，不等于把上游源码全部提交。
- 小型、可公开分发的测试图像和 rosbag；应注明许可证、来源和是否含设备信息。

### 3.2 不应提交或不应作为通用配置提交

- `build/`、`install/`、`log/`、`__pycache__/`、`.ros/` 和编译缓存。
- 带账号、密码、API token、私钥、机器人 IP/控制端口白名单的文件；工控机现场参数放在受控的未跟踪覆盖目录。
- 当前机器专属的 RealSense 序列号、手眼标定结果、TCP/负载和关节零位，除非经过脱敏并作为明确版本化的设备配置发布。
- 实际厂商 SDK 二进制、串口设备节点、系统服务文件和未经许可的上游源码快照。保留可审计的来源地址、许可证和 SHA-256；按许可证决定是否用 submodule、包管理或离线归档。
- Gazebo attachment、`gz_ros2_control`、仿真真值和 fake/mimic JointState 作为实机启动入口。

当前 `.gitignore` 已排除主要生成目录和部分上游源码，但在提交前仍需用 `git status --ignored` 审查。仓库中已有的预存修改/删除不应被本次迁移操作自动清理；提交应使用明确的路径清单和独立 commit。

## 4. 本机与工控机互通建议

工控机同为 Ubuntu 22.04 + ROS 2 并不保证二进制或 ROS graph 兼容。应固定 ROS 发行版、RMW、编译器、依赖版本和仓库 commit，先在无硬件环境构建，再复制到工控机的独立 workspace：

```text
现有厂商 underlay
        ↓（只 source，不覆盖）
独立 rm65b_observer_ws overlay
        ↓
RealSense 只读订阅 / RViz / TF / shadow PlanningScene / MTC planning-only
```

建议流程：

1. 本机提交源码、`*.repos`、参数模板和文档；不要提交工控机私有标定和密钥。
2. 工控机 `git clone` 固定 commit，在独立用户目录编译；保存 `ros2 doctor`、`ros2 pkg list`、RMW 和依赖版本。
3. 先只启动 RealSense 驱动和 observer，检查 RGB、aligned depth、CameraInfo、TF、时间戳、QoS 和 `/object_pose`；不启动第二个 RM driver，不创建 trajectory action client。
4. 用实测 JointState 做 RViz 和 MoveIt 起始状态，执行 MTC planning-only；规划结果只显示和记录，不下发机械臂。
5. 夹爪驱动未验证时，把 gripper backend 设为 disabled/mock，并在任务状态中明确 `gripper_ready=false`；禁止用仿真夹爪反馈冒充实机反馈。
6. 只有厂商 API owner、夹爪协议、急停、限速、停止/取消和读回均单独验收后，才建立 real execution adapter。该 adapter 使用独立 launch/参数和审批，不与仿真控制器并行。

ROS namespace 或 `ROS_DOMAIN_ID` 只能减少 DDS 图发现，不能阻止直接 TCP、串口或厂商 API 写硬件。真正的只读阶段还需要不启动控制节点、不给容器映射设备、必要时使用网络 ACL 阻断控制出站。

## 5. 实机接口验收最小清单

| 层 | 必查内容 | 通过条件 |
|---|---|---|
| RealSense | RGB/depth encoding、aligned depth、CameraInfo、帧率、stamp、QoS | 连续采样无丢帧，RGB/depth 时差在门限内，内参与标定文件一致 |
| TF | `base_link → ... → camera_color_optical_frame` | 唯一发布者，静态量测与 TF 误差在项目门限内 |
| 感知 | 目标 pose、置信度/残差、过期和失败状态 | 静止重复测量可复现，失效时不发布可执行 pose |
| MoveIt/MTC | 实测 start state、碰撞模型、IK、轨迹 | planning-only 成功且不连接硬件 action |
| RM65B API | 模式、使能、报警、停止、读回和单位 | 由设备负责人确认，未通过不得运动 |
| 夹爪 | 型号、协议、开闭反馈、行程和抓取保持 | 未验证时必须禁用自动抓取 |

## 6. 当前判断

已完成标定的 RealSense 和可通过官方 API 控制的 RM65B，适合先迁移“相机 observer、TF/JointState 可视化、无标记感知、PlanningScene 和 MTC 影子规划”。当前工程中 Gazebo、仿真控制器、attachment/contact、fake feedback 和未经验证的夹爪执行不应直接带到实机控制链。夹爪未验收不是 Git 互通的阻塞项，但它是完整 Pick & Place 实机执行的阻塞项；在此之前只能验证视觉、规划和机械臂无夹持的受监督基础动作。

本文是工程迁移设计和风险边界，不构成实机运动授权或安全验收。

# 2026-09-20 ROS 2 Humble / Gazebo Harmonic RM65B 夹爪迁移报告

## 结论

已将当前主工作空间中真正属于项目自建适配层的 RM65B + CTM2F110 仿真功能整理到独立仓库，
目标基线为 Ubuntu 22.04、ROS 2 Humble、Gazebo Harmonic。官方模型、MoveIt、OMPL 和厂商
源码没有复制，而是以版本记录和导入脚本作为依赖。

本迁移提交完成了文件分层、路径去绝对化、Humble 依赖脚本、Harmonic 控制插件构建入口、
迁移清单和运行文档。当前执行环境是 Ubuntu 24.04 / ROS 2 Jazzy，因此本次能完成的是
静态结构检查和脚本审阅；不能把本机 Jazzy 的仿真结果写成 Ubuntu 22.04/Harmonic 验收结果。
22.04 的端到端验收应在目标系统执行 README 中的命令并保存实际日志。

本次已完成的静态证据包括：迁移版 6 个包可被 `colcon list` 发现；在本机 Jazzy 环境中
完成一次编译检查；RM65B + CTM2F110 规划和 Gazebo Xacro 可展开并通过 `check_urdf`。
这些只证明项目文件结构正确，不证明 Humble/Harmonic ABI 或控制插件已在 22.04 上通过。

## 为什么不是直接把当前工程整体复制

当前工程包含三类内容：

1. 项目自建的 RM65B、夹爪、Gazebo、MoveIt 适配层。这些是迁移主体。
2. 官方/成熟库资产，例如 `rm_description`、MoveIt、OMPL、Franka 和测试模型。这些复制会
   造成版本漂移、许可证边界和无意义的仓库膨胀，所以只记录来源。
3. 教学算法、教材、实验历史和生成目录。它们服务于主工作空间学习，不是实机准备所需运行时，
   按用户要求不迁移。

## Humble + Harmonic 的额外工作

Humble 的默认 Gazebo 配对是 Fortress；Harmonic 需要单独安装 OSRF 的 Harmonic 和
`ros-humble-ros-gzharmonic`、bridge、sim 包。`gz_ros2_control` 的 Humble + Harmonic
组合不是默认二进制配对，必须设置 `GZ_VERSION=harmonic` 后从 `humble` 分支构建。
迁移仓库因此提供 `build_gz_ros2_control_harmonic.sh`，并锁定官方 Humble CI 页面可核验的
完整 commit `522ca0e0d3560dc8c52628e9f52b86db78673920`，避免把可变分支静默当作可复现依赖。

## 已迁移的接口关系

```text
厂商 rm_description（上游导入）
             │
             ├── 项目 RM65B SRDF/MoveIt 配置 ── MoveGroup/RViz
             │
             └── 项目 Gazebo Xacro ── gz_ros2_control(Harmonic)
                                             │
                             arm JTC + hand JTC + joint states
                                             │
                                         MoveIt Execute
```

夹爪采用一个主动关节和五个状态联动关节；联动关节不配置命令接口。Gazebo 反馈是实际状态
来源，RViz 只是规划和可视化客户端。组合模型仍使用 CTM2F110，而不是经过确认的 CTM2F110E
实机模型；`Link6` 到夹爪的安装位姿也是项目仿真假设。

## 复现与验收

```bash
cd ~/realman_ws
bash scripts/setup_ubuntu2204_ros2_humble.sh
bash scripts/import_upstream.sh
bash scripts/prepare_realman_model_source.sh
bash scripts/build_gz_ros2_control_harmonic.sh
source .third_party/gz_ros2_control/install/setup.bash
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys="ros_gz ros_gz_bridge ros_gz_sim gz_ros2_control"
colcon build --symlink-install
source install/setup.bash
python3 scripts/verify_migration_manifest.py
bash scripts/verify_environment.sh
ros2 launch manipulation_bringup realman_moveit_gz.launch.py \
  with_gripper:=true
```

启动后另开终端执行两个 `verify_realman_*` 脚本；它们只读取控制器、action 和关节反馈，
不会向实机或仿真发送运动目标。只有在关节反馈确实随轨迹变化、五个 mimic 关节满足倍率，
且 MoveIt 执行日志成功时，才可记录为仿真通过。

## 风险、限制和后续步骤

- 不能把 CTM2F110 资产推断为 CTM2F110E。需要实物铭牌、厂家 CAD、安装孔和惯量资料。
- 需要实测 RM65B `Link6` 到转接板/夹爪基座的平移和旋转，替换零位占位变换。
- 需要在目标 22.04 系统复核 `gz_ros2_control` 完整 commit 和安装包版本，并保存构建日志。
- 需要在没有接入实机的前提下完成 Gazebo、MoveIt、RViz 的回归；然后才制定低速、急停、
  网络隔离、载荷和失联策略。
- 本仓库不包含实机驱动迁移，也不授权任何实机运动。

## 来源

- [RealMan ROS 2 Humble 仓库](https://github.com/RealManRobot/ros2_rm_robot)
- [Gazebo ROS 2 集成说明](https://gazebosim.org/docs/harmonic/ros2_integration/)
- [`gz_ros2_control` Humble 文档](https://control.ros.org/humble/doc/gz_ros2_control/doc/index.html)
- [ChangingTek CTM2F110 产品页](https://en.changingtek.com/diandong/159)

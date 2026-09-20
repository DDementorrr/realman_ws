# 2026-09-20 RM65B + 末端夹爪迁移清点与 Humble/Harmonic 仓库报告

## 结果

已在 `/home/ww/realman_ws_migration` 建立最小迁移仓库，并完成本地提交
`d5436ab`（`迁移 RM65B 夹爪仿真到 ROS2 Humble Harmonic`）。目标仓库是
`https://github.com/DDementorrr/realman_ws.git`，当前远端为空；本次推送因环境没有
GitHub HTTPS/SSH 凭据而未完成。

## 迁移内容

- 自建 `robot_changingtek_description`：CTM2F110 ROS 2 几何、网格和 Xacro；
- 自建 `robot_realman_moveit_config`：RM65B + 夹爪 SRDF、运动学、限位、OMPL 和控制器映射；
- 自建 `robot_gazebo`：RM65B 裸臂回退与 CTM2F110 联合 Gazebo 模型、控制器和 mimic world；
- 自建 `manipulation_bringup`：Humble/Harmonic 启动编排；
- 自建 MoveIt 目标位姿适配器和 RM65B 交互目标输入器；
- 上游导入、Harmonic `gz_ros2_control` 源码构建、环境和运行时验收脚本；
- 中文迁移清单、迁移报告、依赖锁定和仓库工程约束。

## 仅记录、不复制

- 瑞尔曼 `ros2_rm_robot` Humble 分支，完整提交
  `c941b565e4f9174afa36561f143ef5fbbb744750`；
- `gz_ros2_control` Humble + Harmonic 完整提交
  `522ca0e0d3560dc8c52628e9f52b86db78673920`；
- MoveIt、OMPL、RViz、ROS 2 控制器和 Gazebo Harmonic 二进制依赖；
- Panda、Franka、PR2、Fanuc 等学习/测试资产；
- `manipulation_core` 教学算法、教材、构建产物和实验历史。

## 验证

迁移仓库通过静态清单检查、Shell/Python 语法检查、6 个包的 `colcon list`，并在当前
Ubuntu 24.04/Jazzy 环境完成结构性编译；RM65B + CTM2F110 规划和 Gazebo Xacro 通过
`check_urdf`。这些证据不等价于 Ubuntu 22.04/Humble/Harmonic 的端到端验收；目标系统
仍需安装 Harmonic 包、构建 `gz_ros2_control`，再运行仓库内两个运行时检查脚本。

## 安全边界

迁移仓库没有串口/Modbus 实机驱动，不会自动连接或使能 RM65B。CTM2F110E 与导入的
CTM2F110 模型的等同性、法兰安装变换、负载和碰撞参数均待厂家资料和实测确认。

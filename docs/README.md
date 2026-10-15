# 工程文档分类

本目录按用途分为六类，当前工程主线是 Ubuntu 22.04、ROS 2 Humble、Gazebo Harmonic 的
RM65B + CTM2F110 工作空间。Panda、Franka、Jazzy 和教学材料保留为历史阅读资料，
不属于本工程的默认构建链。

| 分类 | 目录 | 内容边界 |
|---|---|---|
| 构建报告 | [`build-reports/`](build-reports/) | 每次目录、依赖、构建或运行验收的日期、命令和结果 |
| 使用指南 | [`guide/`](guide/) | 启动、目录架构、MoveIt、Gazebo 和 SDK 使用方法 |
| 迁移记录 | [`迁移记录/`](迁移记录/) | 迁移清单、来源锁定、排除项和目标环境合同 |
| 实验记录 | [`experiments/`](experiments/) | 可复现的运行观察和故障排查，不替代构建报告 |
| 机器人资料 | [`robots/`](robots/) | RM65B、CTM2F110 规格、来源等级和参数可信度 |
| 教材 | [`textbook/`](textbook/) | 运动学、规划、轨迹和控制的历史学习材料 |

当前工程入口：

1. [`guide/工作空间架构.md`](guide/工作空间架构.md)：`src/` 对象分组和运行链。
2. [`guide/瑞尔曼RM65B与知行夹爪联合仿真.md`](guide/瑞尔曼RM65B与知行夹爪联合仿真.md)：联合仿真与实机边界。
3. [`guide/RM65B_PickPlace长时目标任务分类.md`](guide/RM65B_PickPlace长时目标任务分类.md)：长时 Pick & Place 的分类、接口和阶段门槛。
4. [`guide/RM65B_PickPlace全流程教程.md`](guide/RM65B_PickPlace全流程教程.md)：有界面全流程、接口验证和原理教程。
5. [`build-reports/2026-10-13-RM65B视觉抓放算法资产盘点与实机隔离迁移方案.md`](build-reports/2026-10-13-RM65B视觉抓放算法资产盘点与实机隔离迁移方案.md)：算法成熟度分类、实机复用边界、工控机隔离和分阶段迁移计划。
6. [`build-reports/2026-10-15-RealSense无标记定位与Git实机互通方案.md`](build-reports/2026-10-15-RealSense无标记定位与Git实机互通方案.md)：无 ArUco 感知算法选型、Git 提交边界和工控机只读互通流程。
7. [`build-reports/2026-10-12-PickPlace轨迹评分与腕部相机增量验证.md`](build-reports/2026-10-12-PickPlace轨迹评分与腕部相机增量验证.md)：当前仿真实现和运行证据。
8. [`build-reports/索引.md`](build-reports/索引.md)：按日期查阅构建与验收报告。
9. [`迁移记录/迁移清单.md`](迁移记录/迁移清单.md)：迁移范围和完成条件。

生成目录 `build/`、`install/`、`log/` 不属于文档或源码分类；它们只保存构建运行产物。

# ChangingTek ROS1 资产归档锁定

- 来源：`https://gitee.com/Ice_Panda/changingtek.git`，通过 Gitee `master.zip` 归档端点下载。
- ZIP SHA-256：`8535b4f70ea8292977f3666964405949de3db25c5e2835702436544f0dd70b00`。
- 本仓库已经保存 ROS 2 适配后的网格与 Xacro；ROS1 原始树只在需要审计来源时运行
  `scripts/prepare_changingtek_ros1_snapshot.sh` 重建，且不参与 colcon。
- Git 克隆和 `ls-remote` 在原系统中超时，因此没有可核验的 Git commit；哈希只锁定下载的
  字节快照，不能称作 commit pin。
- 归档包含 `CTM2F110` 资源，不包含独立的 `CTM2F110E` 资源。实机型号等同性仍需厂家图纸确认。

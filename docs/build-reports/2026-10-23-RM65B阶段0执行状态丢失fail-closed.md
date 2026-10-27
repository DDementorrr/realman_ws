# 2026-10-23 RM65B 阶段 0 执行状态丢失 fail-closed

## 结论

本次强化 backend-neutral `ExecutionCoordinator` 的状态丢失和非法状态处置，并增加离线 mock 回归。状态查询失败不再被当作后端已停止；停止确认只接受 `BackendState.CANCELLED`，`BLOCKED` 为锁存故障。改动没有接入 ROS/MTC/Gazebo 执行节点，因此 **G0-6 仍未通过，阶段 0 未通过，阶段 1—6 不开始**。

没有启动 Gazebo 抓放任务、连接 RM65B 实机、提交或推送；工作区既有暂存、未暂存和未跟踪内容予以保留。

## 状态机变化

修改 `src/ros2_adapters/pick_place_runtime/pick_place_runtime/execution_protocol.py`：

- `RUNNING` 中 `poll()` 抛异常时，请求一次 `backend.cancel()` 并进入 `STOPPING`；取消请求自身失败则立即进入 `BLOCKED`。两种情况都不启动下一段。
- `RUNNING` 收到非 `BackendState` 值时按非法状态处理，请求停止并进入 `STOPPING`，而不是把普通字符串或 `None` 当成运行状态。
- `STOPPING` 仅以对象身份等于 `BackendState.CANCELLED` 的反馈作为确认。poll 持续异常、返回 `None` 或返回普通字符串（包括 `"cancelled"`）都不能确认停止；超过 `cancel_timeout` 后进入 `BLOCKED`，诊断中保留查询错误或非法值。
- `BLOCKED` 锁存：`tick()` 不再轮询，`cancel()` 不再发请求，`start()` 抛 `ProtocolError`；未来需设计显式恢复/复位协议后才能清除此状态。
- 原有候选绑定、revision/ready 拒绝、无后端禁止动作、逐段成功确认、后端明确 FAILED 和超时取消路径保留。正常取消确认仍沿用既有 `ExecutionState.FAILED` + `code=cancelled` 表示。

## Mock 回归

`scripts/test_execution_protocol.py`扩展 mock 后端以注入状态查询和取消请求错误。协议共 **19/19** 项通过，新增用例覆盖：状态查询丢失后请求停止、RUNNING 非法状态、STOPPING 中 `None`/伪造字符串状态超时、取消请求失败、持续状态查询错误、取消超时边界，以及 BLOCKED 锁存后不再轮询且拒绝重启。既有候选绑定、无后端、成功分段、明确失败、取消确认和旧 revision 用例继续通过。

## 验证

```bash
bash scripts/check_execution_protocol.sh
bash scripts/check_execution_scene_revision.sh
bash scripts/check_candidate_revision_contract.sh
python3 scripts/verify_migration_manifest.py
bash scripts/verify_environment.sh
python3 -m py_compile src/ros2_adapters/pick_place_runtime/pick_place_runtime/execution_protocol.py scripts/test_execution_protocol.py
git diff --check
```

当前结果：执行协议 **19/19 PASS**；revision 门控 **5/5 PASS**；候选 revision 静态合同 PASS；migration manifest PASS；Humble/Harmonic 环境检查全部 PASS；Python 编译及 whitespace 检查 PASS。

以上是本机 mock、静态和环境可见性证据。没有运行 ROS 节点、MTC 规划、colcon 构建或 Gazebo 任务；环境检查不证明执行链路行为。

## cheap-worker 记录

使用 cheap-worker 复制模式尝试实施，离线 doctor 检查通过。运行 `run-20261023-203940-0039f9f7` 首轮曾到 `READY_FOR_REVIEW`，但人工复核发现 STOPPING 状态收到 `None` 或非法状态时可能在 `status.value` 处抛异常。发起一次返工后，运行因累计模型请求达到上限以 `ERROR` 结束，没有生成可复核的第二轮候选，也没有应用补丁。之后由主会话在原工作区按同一限定范围完成状态机、测试和本报告，并单独运行上述检查；最终通过结果不是 worker 的返工验证结果。

## G0 状态

公共协议现在对状态查询丢失和非法反馈 fail-closed，但仿真执行器仍未接入 `ExecutionCoordinator`；`STOPPING`/`BLOCKED` 尚未映射到真实 ROS action/service 取消、Gazebo 状态反馈或运行期审计记录，也没有完整任务级执行证据。因此 **G0-6 未通过，阶段 0 未通过，阶段 1 不开始**。

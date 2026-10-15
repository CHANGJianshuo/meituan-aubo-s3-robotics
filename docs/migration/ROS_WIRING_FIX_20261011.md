# Python ROS 接线修复与复验（2026-10-11）

实际 Jazzy 启动发现，离线 Mock 通过并不能证明 Python ROS 外壳可启动。本补丁基于
`1d940f7c5f12561fed6641457c0b4b630a282b05`，只修复三个接线问题：

| 问题 | 修复与兼容范围 |
| --- | --- |
| B01 Task 覆盖 `rclpy.Node._clock`，启动报错 | 把 FSM 可调用时钟单独存为 `_fsm_clock`；保留真实 ROS Clock 及原 `clock=` 注入接口 |
| B02 `PickPlaceConfig` 不接受 `perception_max_age_sec` | 映射到实际字段 `perception_max_age_s`；外部 ROS 参数名仍为 `perception_max_age_sec` |
| B04 解锁客户端与工具服务名称不一致 | 默认客户端改为 `/mtc/tool/trigger_unlock`；显式 `unlock_service=` 覆盖接口保留 |

FSM 状态转换、Cancel 内核、Action IDL、四个 C++ 核心包与驱动未修改。集成侧提交补丁，
FSM/Action 负责人复核；B03 及后续生命周期问题继续由模块负责人处理。

## 验证

在独立 Ubuntu 24.04.5 / ROS 2 Jazzy / Python 3.12.3 容器中，以已构建的 12 包工作空间
作为 underlay，另建 `mtc_task`、`mtc_manipulation` 两包 overlay，构建成功。
容器无设备挂载、无端口映射且已断开外部网络，ROS domain 95，发现范围 LOCALHOST。

| 检查 | 结果 |
| --- | --- |
| 同一版新增 ROS 回归在未修复基线上运行 | 6 FAIL，分别复现时钟冲突、配置 TypeError、解锁服务不可达 |
| 新增 ROS 回归在补丁 overlay 上运行 | 6 PASS / 0 FAIL / 0 SKIP |
| 既有 Python 包测试，逐包运行 | 269 PASS / 0 FAIL / 0 SKIP |
| 显式纯 Python Mock E2E | 15 PASS / 0 FAIL / 0 SKIP；使用自带 MockPlanner |
| 实际完整 Python Mock 启动及延迟基础 Goal，两次 | 仍为 FAIL，详见下面的剩余问题 |

新增测试覆盖默认／注入时钟、默认／覆盖的 ROS 感知时效参数，以及真实 DDS 下的工具服务
联锁拒绝与 Mock IO 受理。工具生命周期使用明确标注的合成测试证据；脉冲受理后依然断言
`verified=False`、没有释放确认。配置测试在真实参数和配置对象构造后主动停止，
不绕过剩余 Action 缺陷来宣称 PickPlace 已启动。

已有 C++ 与独立 Fake 驱动代码未变，本轮未重复运行其验收。

构建并 source 工作空间后，从仓库根运行（仅隔离 Mock 环境）：

```bash
export ROS_DOMAIN_ID=95 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export PYTHONNOUSERSITE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
python3 -m pytest -q tests/test_ros_wiring.py

# 各包存在同名 test 模块，分进程运行以避免 pytest 收集命名冲突。
for package in mtc_aubo_bridge mtc_manipulation mtc_motion_execution mtc_safety mtc_task mtc_tool; do
    python3 -m pytest -q "src/$package/test" || exit 1
done
python3 tests/test_mock_end_to_end.py
```

## 剩余问题与复核条件

- **B03：PickPlace ActionServer 绑定。** `PickPlaceServer` 仍使用 `action_type.Server(...)`
  和 `action_type.CancelResponse()`；生成 Action 类型没有这些绑定接口。此路径被 B05 遮挡，
  本轮未修改，也未声称已运行到该行；需负责人接入真实 ActionServer 并验 Goal/Cancel/Result。
- **B05：运动 ActionClient 绑定。** B02 修复后，真实启动进一步到
  `RosMotionPort.__init__`，在 `action_type.create_client(...)` 报
  `AttributeError: type object 'ExecuteJointMove' has no attribute 'create_client'`。
  两次启动复现，PickPlace 进程退出 1；需负责人接入真实 ActionClient 并核对结果与取消行为。
- **B06：等待首个任务时的 INIT 生命周期。** 两次均在启动约 5 秒后、发送 Goal 前出现
  `state INIT timed out ... -> HARDWARE_FAULT`。随后基础 Goal 被受理，收到 FAULT 反馈，
  最终 `ABORTED`（6）、`success=False`、`error_code=510`、完成数量 0。
  这只证明延迟发送的测试路径失败；需负责人决定初始化／待命与看门狗契约并覆盖延迟首次 Goal。
- **退出观测：** SIGINT 停止时 `safety_supervisor` 出现 `rcl_shutdown already called`，
  其进程退出 1；保留为退出清理问题。launch 及日志采集退出 0 不代表任务成功。

第二次检查通过逐节点 Action server 列表及 `wait_for_server` 确认：Task 和 Motion
服务器可用，PickPlace 服务器不可用。通用 Action 列表会包含 Task 的 PickPlace **客户端**，
不能据此认定 PickPlace 服务启动成功。两次 Task Goal 使用黄色作为验收示例，保持默认
`dry_run=True`；不代表真实抓放、MoveIt 或新 C++ FSM 闭环通过。

生成式 AI 披露：使用 OpenAI Codex 编写三项接线补丁、ROS 回归及验收编排并整理复验结果。
生产变更仅涉及上述两个 Python 外壳文件，原有许可证与第三方来源约定保持适用。

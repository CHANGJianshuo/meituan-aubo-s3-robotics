# C++ Mock Bringup 与 Web UI

本入口把实际 C++ `TaskServer`、C++ `PickPlaceServer` 与网页放在同一 ROS 2 命名空间。网页经 rclpy 发送 `ExecuteTask`；Task 经已有 C++ `rclcpp_action::Client<PickPlace>` 调用下游。不是 Python Action Server 或网页定时演示器。

候选起点：`F1u0rite/meituan_challenge_ws@f7eaa2c0d2887aeada4fd635112c63c237962ff7`（2026-10-12 08:00 北京时间截止时的 main）。没有合入未合并的轨迹执行、视觉、标定或 Planning Scene 分支。本轮只增加启动、参数、只读观测、UUID 日志和 Web 接入；不改 FSM、ActionSlot、Cancel 判断或 Motion Execution 内核。正式项目 pin `1d940f7…` 不因本 PR 推进。

本候选与验收源版本、PR、日志索引的对应关系见 [本轮验收报告](https://github.com/AfonsoZhang/meituan-robotics-challenge-2026/blob/main/docs/cpp-mock-webui-acceptance.md)。报告在独立验收仓库归档，原始失败日志保留。

## 环境与构建

Ubuntu 24.04、ROS 2 Jazzy、C++17。使用已有 Jazzy 依赖缓存/镜像；构建需要 `rclcpp`、`rclcpp_action`、`rclpy`、`std_msgs`、`action_msgs`、`sensor_msgs`、`rcl_interfaces`、`ament_index_python`、`ament_cmake_gtest` 和本仓库 `mtc_interfaces`、C++ Core/Task/Manipulation/Motion 包。Web 使用 Python 标准库与本地 HTML/CSS/JS，无 npm/CDN。相机功能不在本轮 Mock 链路中。

```bash
source /opt/ros/jazzy/setup.bash
cd /path/to/meituan_challenge_ws
git rev-parse HEAD                 # 与验收报告的候选提交核对
export MAKEFLAGS='-j2 -l2'
colcon build --packages-up-to mtc_cpp_mock_bringup --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
source install/setup.bash
```

本项目验收在单个隔离容器内完成：`--network none --cpus 2 --memory 4g --pids-limit 512 --cap-drop ALL --security-opt no-new-privileges`；无设备映射、宿主挂载、Docker 端口映射。源码用 `docker cp` 复制，容器不取网、不接实体设备。镜像摘要与安装版本以验收报告和环境日志为准，不冒充队友电脑/NUC 的版本。

## 统一启动

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/meituan_challenge_ws/install/setup.bash
export ROS_DOMAIN_ID=161
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_STATIC_PEERS ROS_LOCALHOST_ONLY FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI
ros2 run mtc_cpp_mock_bringup mock_stack \
  --namespace /mtc_cpp_mock --output-dir /tmp/cpp-mock-run-001 --port 8767
```

输出目录必须是新目录；拒绝覆盖。看到 `READY` 后，浏览器打开 `http://127.0.0.1:8767/`，页面标为“C++ Mock · ROS 2”。`--no-web` 只起 C++ 节点；`--readonly` 禁用 Web 写操作；`--result-delay-ms` 调整已有软件 Mock Future 交付延时，默认 250 ms，不是机械运动时间。`--profile success|planning_failure|cancel_unconfirmed|status_unknown` 仅在启动时选择已有 Mock 选项，网页不能修改故障配置。

Mock 感知、规划、工具、运动适配器在 C++ PickPlace/Task 中；不需启动 Python Task/PickPlace、Motion Action Server、Driver 或 MoveIt。后者不能因本轮通过而宣称接通。启动程序持有 domain/namespace 文件锁，检查旧默认 Action 地址和本命名空间，发现占用则退出。不会杀其他进程或清空 ROS domain。

## 接口、就绪与冲突检查

| 角色 | 默认节点 / Action |
| --- | --- |
| C++ Task | `/mtc_cpp_mock/mtc_mock_task`；`/mtc_cpp_mock/task/execute`，`mtc_interfaces/action/ExecuteTask` |
| C++ PickPlace | `/mtc_cpp_mock/mtc_mock_pick_place`；`/mtc_cpp_mock/manipulation/pick_place`，`mtc_interfaces/action/PickPlace` |
| Web 桥接 | `/mtc_cpp_mock/mtc_web_console_<session>`，只向 Task 发任务 |
| 只读就绪 | 两个 C++ 节点各自的 `~/integration_state`，`std_msgs/msg/String` JSON，Reliable / Volatile / depth 5，100 ms 周期 |

```bash
ros2 run mtc_cpp_mock_bringup mock_check --namespace /mtc_cpp_mock \
  --output /tmp/cpp-mock-ready-001.json
```

检查要求两个 Action 各有唯一的指定 C++ 服务端、类型正确、Task 的下游 ActionClient 存在；两份只读状态来自唯一指定发布节点、时间戳新鲜、软件自检完成且 admission 开放。就绪状态反映已有 slot/FSM 的值，不改 admission 或内核状态。图快照含节点、客户端、Action、Topic 类型、发布端 GID 和 QoS。它是联调来源检查，不是抵御恶意 ROS 节点伪装的认证机制。

网页不生成 RobotState/ToolState，也不假称机器人通信或独立停稳已确认。它只允许明确的 `cpp_mock` 模式根据 C++ 就绪状态工作；原 ROS 实机模式仍保留实机状态门禁。旧 `--mock` 模式属于独立网页定时演示器，不构成本轮证据。

## 发任务与取消

在网页选择颜色/模式后提交；HTTP 202 表示排队，必须随后看到 ROS Goal 受理和实际 Feedback。结果页/会话导出保留 task_id、Goal UUID、错误码、完成数和事件时刻。

也可用带原始父子 Action 观测的客户端：

```bash
ros2 run mtc_cpp_mock_bringup mock_task --namespace /mtc_cpp_mock \
  --output-dir /tmp/cpp-task-basic-001 \
  --goal '{"task_id":"basic_001","mode":1,"ordered_colors":[1],"timeout_ms":30000,"placement_stability_sec":3}'

ros2 run mtc_cpp_mock_bringup mock_task --namespace /mtc_cpp_mock \
  --output-dir /tmp/cpp-task-cancel-001 --cancel-at MOVE_PRE_ALIGN \
  --goal '{"task_id":"cancel_001","mode":1,"ordered_colors":[2],"timeout_ms":30000,"placement_stability_sec":3}'
```

命令中的颜色只是测试输入。序列模式为 `mode=2` 和三个不同颜色；不是正式比赛口令。任务 ID 在同一 Server 生命周期中不能重复使用。

网页 Cancel 只取消本会话拥有的 Goal UUID。刷新不会重新发 Goal；重启 Web 会话没有旧 Goal 的句柄，不能代替原会话取消或恢复任务。Cancel ACK 与最终 Result 分开显示；Task 只有收到关联 PickPlace 的实际终态才作原内核判定。`CANCELED/0` 为软件 Mock 取消完成；`ABORTED/520` 为取消证据不足，不能当作停稳。错误 230/520 后的 admission lock 不由网页清除；转交孙嘉明处理内核恢复契约。本工具不会自动重启服务来绕过锁。

> 已知未通过：正常 Mock 配置下，浏览器可见 MOVE_PRE_ALIGN 后较晚取消会得到 ABORTED/520；及时 HTTP 取消有 CANCELED/0 正例。详见 [C1 最小复现和负责人交接](../../docs/migration/CPP_MOCK_WEBUI_REVIEW_20261012.md)。这限制了本轮取消完成结论，不能据此宣称完整通过。

## 日志与停止、重启

启动目录包含 `launch.json`（实际 argv/PID/可执行文件）、`preflight.json`、`ready.json`、`web-ready.json`、两份 C++ 日志、`web.log`、`web-session/`、`cleanup.json`、`cleanup-graph.json`。C++ `MTC_TRACE` 记录 Task UUID、子 Goal UUID、request_id、Result/status/error 和定向 Cancel。客户端目录包含父/子状态与 Feedback 的 `events.jsonl` 和真实 Result；未收到 Result 保持 UNKNOWN。

在启动终端 Ctrl+C，或对 `mock_stack` PID 发 SIGTERM。它只终止自有进程组，等待退出并检查图排空。8 秒未退出会记录强制清理并返回失败；不得把该结果当作正常清理通过。退出 UI 不会自动制造任务取消结果，停止整套 Mock 也不能当作完整的 Cancel 验收。

```bash
ros2 run mtc_cpp_mock_bringup mock_check --namespace /mtc_cpp_mock --empty \
  --output /tmp/cpp-mock-empty-001.json
# 使用全新日志目录重新启动同一 namespace；不覆盖旧证据。
```

## 复现与常见故障

```bash
# C++ 原有真实父子 Action 回归
colcon test --packages-select mtc_ros2_action_cpp --return-code-on-test-failure
colcon test-result --verbose
# 串行启动真实 C++ 节点，经 HTTP/ROS 检查正负路径并逐次清理
python3 -m mtc_cpp_mock_bringup.acceptance --output-dir /tmp/cpp-http-acceptance-001
```

- `NOT_READY`：查看两个 C++ 只读状态的来源/龄期/admission、Action 数量和类型。不要补造 RobotState 绕过门禁。
- HTTP 202 后 REJECTED：有其他 ROS 客户端先受理、Goal 不合法、ID 重复或 slot 锁定；以 ROS 回执为准，不自动重发。
- 采集不到短任务：先等客户端 server discovery 和父/子反馈及状态订阅匹配；固定睡眠不能代替该检查。
- `--profile planning_failure`：错误 200 是故障注入预期结果；`cancel_unconfirmed` 的 ACK 后应检查最终 520；`status_unknown` 检查 230 和锁定。
- 看见旧 Python Server、同名节点或多个 Action Server：停止本次启动并定位其所属进程，保留冲突快照；不使用 `pkill` 或 reset 内核冒充修复。
- UI 未显示相机、机器人或工具状态：本轮 C++ Mock 没有这些发布源，未知状态是如实展示。

使用 OpenAI Codex 协助实现和验收。移入 Web 代码的确切来源、原文件哈希、依赖许可证与本轮修改边界见 `provenance.json`、`mtc_webui/provenance.json`；未替第三方代码重新授权。

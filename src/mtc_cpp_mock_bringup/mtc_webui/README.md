# Web 客户端模块

统一 C++ Mock 的构建、启动和验收见 [上级 SOP](../README.md)。该模块名为 `mtc_webui`，避免与调用者当前目录中的旧 `webui` 遮蔽。

`server.py` 提供只监听 loopback 的 HTTP 服务；默认 ROS 客户端模式保留 Task/Robot/Tool 来源、时间戳、实机状态门禁。明确的 `--cpp-mock` 才使用本轮 C++ 节点只读状态，并发送实际 ROS ExecuteTask Goal。另一个 `--mock` 是历史网页定时演示器，不连接 ROS，不能当作本轮通过证据。

只在本轮 C++ Mock 模式中验收了网页 Goal/Feedback/Result/Cancel。实机门禁、相机/关节显示的历史接口测试说明保存在原始 `provenance.json`，不表示它们已接入本候选。原个人仓库导入版本与哈希、本轮修改归属见 [来源记录](../provenance.json)。

```bash
# 已 source Jazzy 和当前候选 install/setup.bash 后
ros2 run mtc_cpp_mock_bringup web_console --cpp-mock \
  --cpp-namespace /mtc_cpp_mock --ros-domain-id 161 \
  --output-dir /tmp/cpp-ui-session-001
```

此命令仅连接已启动的 C++ 节点；完整启动请使用 `mock_stack`。输出目录必须新建。取消仅针对本会话拥有的 Goal，ACK 不代表最终取消，未知结果不会重发。`test_*.py` 为离线行为检查；真实 C++ 链路由上级 acceptance 工具和浏览器验收记录证明。

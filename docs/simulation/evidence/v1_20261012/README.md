# V1 验证证据

- `build.log`：独立 `install/v1_sim` 构建记录。
- `regression.log` / `test_results.txt`：五个相关包的测试结果；colcon 的 321 项结果包含测试套件汇总。
  实际测试用例为 284 项 C++ GTest 和 14 项 Python 测试。
- `versions.txt`：本机 ROS / MoveIt / Gazebo 控制组件版本。
- `source_manifest.json` / `binary_manifest.json`：验证时的源文件与二进制 SHA-256。
- `basic_A/`：基础 A→T0 的结果、原始轨迹、完整发送命令、相机快照和位置曲线。
- `sequence_CAB/`：顺序 C→P1、A→P2、B→P3 的同类证据。
- `pause_fault/`：运行中暂停 Gazebo 物理；应取消当前轨迹、进入 FAULT、拒绝宣称放置成功。
- `refuse_real_config/`：`simulation_only=false` 时拒绝创建运动链路。

原始 `physics.jsonl` 和 `commands.jsonl` 无截取地压缩保存。视频与完整 ROS 日志在工作区
`outputs/v1_validation_20261012/`，不提交大体积运行文件。

独立复算示例，不需要 ROS 节点或仿真进程：

```bash
run_dir=$(mktemp -d)
cp docs/simulation/evidence/v1_20261012/basic_A/*.json "$run_dir/"
gzip -dc docs/simulation/evidence/v1_20261012/basic_A/physics.jsonl.gz > "$run_dir/physics.jsonl"
python3 src/mtc_simulation/scripts/analyze_v1.py "$run_dir"
```

`pause_fault/run.json` 中的总体成功和 `shutdown_clean` 为 false：任务节点按预期以 1 退出，
启动器保留该失败。该项是否符合预期以 `fault_check.json` 和取消/FAULT 事件判定，不能计作
正常抓放成功。原始历史运行路径保留在 JSON 中，用于与日志对应。

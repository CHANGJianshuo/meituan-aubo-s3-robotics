# mtc_bringup —— 启动、参数组合与离线配置校验

本包是 AUBO S3 比赛工程的**启动编排 + 配置校验**层（README §5 结构中的
`src/mtc_bringup/`）。

> **安全默认值：Mock / 离线。** 本包不会连接真实 AUBO S3，不会启动任何真实设备节点，
> 也不会触发电磁铁。`real_bringup.launch.py` 存在但**默认拒绝启动**。

## 1. 内容

| 路径 | 作用 |
| --- | --- |
| `launch/mock_bringup.launch.py` | 离线 Mock 组合启动；打印“Mock 模式，未连接真实 AUBO S3”告警 |
| `launch/sim_bringup.launch.py` | V1 Gazebo 仿真入口，需构建 `mtc_simulation`，见该包 README |
| `launch/real_bringup.launch.py` | 真实组合入口；除 `i_understand_real_robot_is_enabled:=true` 外一律拒绝启动，且**即使确认也不连接设备** |
| `mtc_bringup/config_loader.py` | **纯 Python、不 import rclpy** 的配置加载与校验器 |
| `mtc_bringup/validate_config.py` | 配置校验 CLI（退出码 0 通过 / 1 失败） |
| `mtc_bringup/bringup_mock.py` | `bringup_mock` 入口点：校验配置 + 打印 Mock 组合 + 可选离线自检 |
| `mtc_bringup/mock_pipeline.py` | 离线 Mock 链路装配（端口适配 + Task/PickPlace 编排），供自检与端到端测试复用 |

## 2. 运行方式（无需 ROS 2，无需 colcon build）

```bash
cd /home/robot/meituan_challenge_ws

# 配置校验（退出码 0 通过 / 1 失败）
PYTHONPATH=src/mtc_bringup python3 -m mtc_bringup.validate_config

# Mock 组合 + 离线自检（蓝→红→黄）
PYTHONPATH=src/mtc_bringup python3 -m mtc_bringup.bringup_mock
PYTHONPATH=src/mtc_bringup python3 -m mtc_bringup.bringup_mock --dry-run

# 离线端到端集成测试
python3 tests/test_mock_end_to_end.py
```

安装后（`colcon build` + `source install/setup.bash`）可直接使用入口点：

```bash
ros2 launch mtc_bringup mock_bringup.launch.py
validate_config
```

> `colcon build` 本机**未执行**（无 ROS 2 Jazzy）。上述 launch 与入口点按 ROS 2
> ament_python 规范编写，**尚未在任何 ROS 2 发行版上完成编译/安装验证**。

## 3. YAML 依赖说明（重要）

`config_loader` **只**依赖 PyYAML。启动时先 `import yaml` 探测：

* **PyYAML 可用** → 使用 `yaml.safe_load`；
* **PyYAML 不可用** → 回退到本包自带的**最小 YAML 子集解析器** `parse_yaml_minimal`，
  **不引入任何第三方库**。

最小解析器支持：注释、空行、嵌套映射、块序列（`- item`）、行内/跨行 `[...]` 与 `{...}`、
标量（str/int/float/bool/null）、引号字符串。**不支持**锚点/别名、`|`/`>` 折叠标量、
制表符缩进——遇到这些结构会**抛出 `ValueError` 而不是静默猜测**（避免把危险数值
误读成默认值）。当前 `config/*.yaml` 在该子集内，两种后端的解析结果已逐一比对一致。

校验输出会显示实际使用的后端（`YAML 解析器 : pyyaml` 或 `minimal`）。

## 4. 校验内容（设计文档 §8）

* 必填字段与取值范围（`placement_stability_sec > 0`、`perception_max_age_sec > 0`、
  `timeout_ms > 0`、缩放因子 ∈ (0, 1] 等）；
* 互斥组合：`tool_type` ∈ {`passive_hook_v1`, `magnetic_latch_v2`}；`mode` ∈ {basic, sequence}；
  颜色枚举合法（0=UNKNOWN 不得作为目标颜色）；序列任务必须 3 色互异且严格映射 P1/P2/P3；
* 联锁开关不得放宽（未挂载禁止运输、未落座禁止解锁、禁止自动重发、停止未确认禁止下一项等）；
* V1 不得声明任何电磁 IO 通道（解锁 IO 调用次数必须为 0）；V2 必须要求落座 + 卸载 + 独立解锁证据；
* 禁止声明未实测的轨迹/笛卡尔能力；
* 凭据扫描：配置中不得出现 password/token/secret 等字段名或 IPv4 字面量；
* `config/safety.yaml` 的设备身份白名单**留空占位**：留空时 `AuboBridge.identity_check()`
  必然不通过，按安全默认**拒绝建立运动通路**。

## 5. 未实现点（如实登记）

* 真实 AUBO S3 通路：未实现、未验收，始终 `disabled`；
* Gazebo / 仿真组合：`NOT RUN`（缺 ROS 2 Jazzy 与 `mtc_simulation` 场景）；
* `mtc_motion_planning`：迁移占位包（源缺失），Mock 链路使用显式标注的 `MockPlanner` 替身；
* 感知：`/mtc/perception/batteries` 尚无实现节点，Mock 链路使用显式标注的 `MockPerception` 替身；
* `mtc_safety/safety_supervisor`、`mtc_aubo_bridge/bridge_node` 节点外壳是否就绪需以实际文件为准，
  `mock_bringup.launch.py` 对未就绪节点打印 `SKIPPED(...)` 而不是静默失败。

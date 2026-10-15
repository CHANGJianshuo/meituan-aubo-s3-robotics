# AUBO S3 电池搬运：任务一、任务二与 Web UI

用 **AUBO S3 六轴机械臂 + V1 被动舌片工具** 完成电池搬运。项目覆盖场景测量、Gazebo 接触仿真、轨迹平滑、实机执行及网页操作。

这是 `meituan_challenge_ws` 的最新源码公开快照，包含未提交的现场改动，以及原 `outputs/` 中的 Web UI 和执行器源码。现场版本日期：**2026-10-14（America/Chicago）**；最新任务二完成记录为 **2026-10-15 01:33 UTC**。

## 文档导航

| 文档 | 内容 |
|---|---|
| [简洁讲解版](docs/QUICK_OVERVIEW.md) | 快速了解做了什么、采用什么方案、结果如何 |
| [详细版](docs/DETAILED_GUIDE.md) | 架构、动作、测量、仿真、提速、实机和 UI、复现步骤 |
| [实机源码与配置](hardware/README.md) | 播放器、SDK、私有配置与登录入口 |
| [发布验证记录](docs/release/VALIDATION.md) | 此公开快照的检查结果与验证范围 |

新说明均为 **Markdown**。旧设计与历史报告保留，当前状态以本页及两份新文档为准。

## 已完成

- **任务一**：红块 → T0；一次启动完成 10 段搬运、卸载、退出。平滑版完整执行时间 **90.439 s**，上一轮为 175.872 s。
- **任务二**：红 → 黄 → 蓝，即 **ABC**，依次放入 **P1 → P2 → P3**；保持舌片参考朝向，绿块留在起始区。
- **最新任务二实机**：**30/30 段成功**、播放器退出码 0、无自动重试；网页单调时钟记录 **214.525 s**，轨迹纯运动时间 **114.009 s**。最终最大关节目标误差约 **0.00384°**，相机确认落位稳定 **3.17 s**。[结果摘要](docs/release/task2_latest_run.json)
- **Web UI**：任务提交、阶段进度、相机预览、实机状态、停止请求、结果和会话记录。

“四倍速”是现场版本名称：最终轨迹运动时间为此前二倍速版本的 1/2；检查和停稳等待另计。关节误差与图像稳定指标不代表电池的毫米级落位精度。

## 方案

```text
RGB-D / 控制柜只读状态 → 场景与起点核对 → MoveIt / Gazebo 接触验证
                                              ↓
                                     平滑重定时 + 原样回放
                                              ↓
浏览器 → 本机 Web UI → SSH / NUC → AUBO SDK 轨迹播放 → 机械臂
             ↑                          ↓
        相机落位核验 ← 事件日志 / RPC / RTDE 反馈
```

实机执行**经过验证的保存轨迹**。C++ FSM / ROS Action、Gazebo 和现场 SDK 播放是不同运行路径；当前完成范围是已测量固定场景的搬运流程。

最新现场播放器按操作者选择将**关节跟踪误差幅值改为只记录**。起终点、停稳、状态、限位、轨迹范围、独占锁和单次执行检查仍在，控制柜自身保护没有由此取消。打包器默认仍为 0.5°，详见[详细版](docs/DETAILED_GUIDE.md)。

## 先在电脑上运行

只演示网页，不连接 ROS 或机械臂：

```bash
git clone https://github.com/CHANGJianshuo/meituan-aubo-s3-robotics.git
cd meituan-aubo-s3-robotics
bash scripts/start_demo.sh --port 8877 --output-dir /tmp/mtc-web-demo-001
```

打开 <http://127.0.0.1:8877/>。每次使用新输出目录；网页明确标记演示模式，演示结果不作为实机结果。

标准库离线检查公开保存轨迹：

```bash
python3 scripts/validate_public_trajectories.py
```

C++ Mock 构建（Ubuntu 24.04 / ROS 2 Jazzy，先准备包依赖）：

```bash
source /opt/ros/jazzy/setup.bash
MAKEFLAGS='-j2' colcon build --packages-up-to mtc_cpp_mock_bringup \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
source install/setup.bash
```

Gazebo 依赖见[仿真包说明](src/mtc_simulation/README.md)，完整步骤见[详细版](docs/DETAILED_GUIDE.md)。实机需要本站点测量、身份、执行包和私有配置，公开示例不含直接连接原现场的配置。

## 代码地图

| 路径 | 用途 |
|---|---|
| `src/mtc_*` | ROS 2 接口、Python/C++ FSM、运动执行、规划、工具与安全模块 |
| `src/mtc_simulation/` | 工具、机械臂/电池/底图资产、Gazebo 仿真与验收 |
| `src/mtc_cpp_mock_bringup/mtc_webui/` | 最新网页及 Mock、ROS、硬件后端 |
| `scripts/task1_real_scene/` | 任务一测量、运动学匹配、导出与平滑 |
| `scripts/task2_real_scene/` | 任务二点位、场景、打包、分段调试和复位轨迹组合 |
| `hardware/` | 各策略播放器、远程桥、SDK 自编封装、配置模板 |
| `examples/field/` | 两个成功版本的轨迹数值副本，仅供离线研究 |
| `tests/` | 离线回归测试 |
| `archive/` | 现场各轮生成源码及合并前差异，不参与 colcon 构建 |
| `docs/release/source_manifest.json` | 逐文件来源、散列和整理变更 |

已收录项目源码、模型、配置、测试和现场生成源文件。未收录构建安装目录、缓存、现场原始录像、运行锁、凭据和私有设备配置。原含口令的本地手册已用公开说明替换。

保留原项目与第三方声明，见 [NOTICE.md](NOTICE.md)。

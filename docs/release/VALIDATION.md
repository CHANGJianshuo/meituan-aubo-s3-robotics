# 公开快照验证记录

本次验证针对 `meituan-aubo-s3-robotics` 的独立整理副本。没有为发布重新驱动真实机械臂，也没有把原现场服务切换到这个副本。

## 结果

| 检查 | 结果 | 说明 |
|---|---:|---|
| 任务二现场脚本与 Web UI | 159 passed | 打包、分段、轨迹导出/平滑、放置证据、各 UI 后端 |
| 发布版 SSH 配置回归 | 6 passed | 必须显式提供主机，保留主机密钥检查，密码不进入 argv |
| 最新 monitor 播放器 | 27 passed | 假控制器异常、轨迹、只记录策略、停止与台账 |
| 0.5° 基线播放器 | 24 passed | 独立目录测试 |
| J4 0.6° 播放器 | 27 passed | 独立目录测试 |
| 全轴 0.6° 播放器 | 27 passed | 独立目录测试 |
| SDK 自编封装 | 39 passed | 假控制器，不连接 SDK 设备 |
| C++/ROS 构建 | 7 packages finished | 最新 Web UI overlay 与根源码合并后重新构建 |
| C++ Core/Motion/Task/Manipulation/Action | 386 tests，0 errors/failures/skipped | colcon test-result 报告口径；5 个受测包 |
| Python 语法 | 通过 | `src scripts hardware` compileall |
| 公开轨迹 | 两份通过 | SHA-256、单位、轴序、连续性、端点静止、时间与样本数值 |
| 独立网页启动 | 通过 | Mock 页面/状态 HTTP 200，错误 Host 返回 403，未连接设备 |

机器可读汇总见 [software_validation.json](software_validation.json)。以上播放器和 Mock 测试输出中的“move/stop/完成”来自假控制器，不能当成新的实机动作。

## 可复现的主要命令

```bash
NO_PROXY='*' no_proxy='*' PYTHONPATH=src/mtc_cpp_mock_bringup \
  python3 -m pytest -q tests/test_task2_*.py \
  src/mtc_cpp_mock_bringup/mtc_webui --import-mode=importlib
PYTHONPATH=src/mtc_cpp_mock_bringup python3 -m pytest -q tests/test_publication_config.py
python3 scripts/validate_public_trajectories.py
python3 -m compileall -q src scripts hardware
```

四个播放器各自在对应目录运行 `python3 -m unittest -q test_runner`。SDK 封装在 `hardware/sdk_helpers/` 运行 `python3 -m unittest -q test_smoke test_j6_45`。

```bash
source /opt/ros/jazzy/setup.bash
MAKEFLAGS='-j2' colcon build --packages-up-to mtc_cpp_mock_bringup \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
source install/setup.bash
ROS_DOMAIN_ID=171 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST colcon test \
  --packages-select mtc_core_cpp mtc_motion_execution_cpp mtc_task_cpp \
  mtc_manipulation_cpp mtc_ros2_action_cpp \
  --executor sequential --return-code-on-test-failure
colcon test-result --test-result-base build
```

## 实机证据与发布边界

最新任务二在原现场版本中已成功完成。发布整理额外核对原记录：动作和完成阶段严格为 1–30 的连续序列，总完成/播放器返回各一次，退出码 0，最终视觉稳定超过 3 秒。[摘要与原记录散列](task2_latest_run.json)

任务一结果来自平滑提速版现场结果记录，公开摘录见 [task1_success.json](task1_success.json)。两份轨迹示例的数值与成功版本逐项相同；修改的只是离线发布元数据，见 [轨迹清单](../../examples/field/manifest.json)。

此轮没有重新运行完整 Gazebo 回放、未重测真实负载/受力或绝对标定，也没有让发布版 SSH 配置改动直接控制现场硬件。历史仿真结果不能替代新站点验收。

## 公开内容检查

扫描了 Token/密钥模式、口令赋值、现场地址，以及包含口令说明的文本与已有 DOCX XML。测试夹具中的 `sentinel-password` / `not-real` 是固定假值。旧本地设备手册含现场口令，公开副本已改为说明页；未上传凭据、设备序列号、原始相机录制、执行台账和私有部署 manifest。

原始工作区保持原样。公开地址替换、可配置入口与旧版本保留情况由 [source_manifest.json](source_manifest.json) 追踪。

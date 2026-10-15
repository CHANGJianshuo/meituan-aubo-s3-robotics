# 现场硬件源码与配置

此目录集中保存与最新 Web UI 配套的 NUC 远程桥、轨迹播放器和 SDK 自编封装。现场任务的整体说明见 [详细版](../docs/DETAILED_GUIDE.md)。

## 文件与入口

| 路径 | 用途 |
|---|---|
| `login_webui.py` | 读取私有配置，交互式登录，启动网页；连接时只读取状态 |
| `remote_bridge.py` | NUC 侧检查包和代码哈希，传递事件，监督心跳/停止 |
| `players/tracking_monitor/` | 最新实际运行策略：跟踪误差只记录 |
| `players/guarded_05/` | 基线策略：六轴 0.5°；打包器默认 |
| `players/j4_06/` | J4 0.6°、其余 0.5° |
| `players/all_06/` | 六轴 0.6° |
| `sdk_helpers/` | 自编 SDK 连接、状态和 RTDE 封装，另附离线测试 |

`sdk_helpers/j6_45.py` 的旧点动命令不是任务一/二入口；只复用其 `Robot45` 类。历史默认配置与说明不能替代本站点实测值。

## 运行依赖

- 本机 UI：Python 3、NumPy、OpenCV；源码路径 `src/mtc_cpp_mock_bringup/`。
- NUC：现场使用 Python 3.10、`pyaubo-sdk==0.24.1`。SDK 需单独安装，未复制其二进制。
- 已核对的 SSH 主机密钥、有效 SDK 登录、本站点执行包与预检证据。
- 现有相机预览进程需提供图像、时间戳及采集契约。这个发布快照未打包原现场的私有相机服务数据。

播放器通过 `MTC_SDK_HELPERS` 查找封装；未设置时按仓库布局寻找 `hardware/sdk_helpers/`。部署到独立包目录时，应在 NUC 会话或远端启动环境中设置绝对路径。`MTC_HARDWARE_PYTHON` 供 `run.sh` 选择 Python，直接由远程桥启动时使用配置里的 `remote_python`。

```bash
export MTC_SDK_HELPERS=/path/to/meituan-aubo-s3-robotics/hardware/sdk_helpers
export MTC_HARDWARE_PYTHON=/path/to/sdk-environment/bin/python
```

## 私有 UI 配置

复制 `hardware_config.example.json` 为仓库根目录的 `hardware_config.local.json`。示例中的 `.invalid`、占位哈希和路径不能用于执行；不要把现场口令写入该文件。

| 字段 | 含义 |
|---|---|
| `ssh_host` | NUC 的 `user@host`，公开版本不内置现场地址 |
| `ssh_host_key_alias` | 可选，仅在 known_hosts 采用别名时设置 |
| `remote_python` | NUC SDK Python 的绝对路径 |
| `remote_bridge` / `remote_bridge_sha256` | 已部署远程桥及其实际散列 |
| `remote_package` | NUC 已核验执行包目录 |
| `package` | 本机同一执行包目录 |
| `manifest_sha256` | 实际 manifest 文件的 SHA-256 |
| `source_trajectory_sha256` | 已验收来源轨迹散列 |
| `preview_dir` | 本机相机预览目录 |
| `preflight_file` | 绑定当前 plan_id、manifest、场景及有效期的预检证据 |
| `claim_file` | 本次网页单次提交登记文件 |
| `placement_reference` / 对应散列 | 经核对的放置参考图 |
| `placement_checks` | 本次视角各对象的 ROI、颜色掩码下标、最小像素数 |

示例 `placement_checks=[]` 会缺少真实验收条件；必须用本站点核对过的区域替换。完整执行包由 `scripts/task2_real_scene/prepare_fullflow.py` 生成，需要保存轨迹、回放验收、平滑验证、硬件参考 manifest 和匹配播放器。它不会从公开轨迹示例自动制造现场验收证据。

配置、部署与预检准备好后：

```bash
python3 hardware/login_webui.py --config hardware_config.local.json \
  --output-dir outputs/webui-session-001 --port 8767
```

账号与密码在终端输入，密码隐藏，通过 stdin 传入服务进程。登录只连接状态；任务由操作者在网页提交。完整包单次使用，下一轮需新执行标识与现场复位核验，不能清空台账重发。

## 离线测试

```bash
cd hardware/players/tracking_monitor
python3 -m unittest -q test_runner
```

这些测试使用假控制器，不连接设备。四个播放器应分别测试，避免 `run_hardware` 同名模块缓存冲突。

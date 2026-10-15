# 现场成功轨迹的离线数值副本

| 文件 | 对应版本 |
|---|---|
| `task1_red_t0_trajectory.json` | 任务一平滑提速版，红 → T0，10 段 |
| `task2_abc_4x_trajectory.json` | 任务二原始起点，红/黄/蓝 → P1/P2/P3，30 段 |

保留所有关节位置、速度、加速度、时间数值；移除现场部署身份与路径。每段从 t=0 开始，位置 rad、速度 rad/s、加速度 rad/s²、时间 s。总时间是各段最后时刻之和。

`manifest.json` 记录来源、原始散列和公开副本散列。公开副本的整体散列不同，因为元数据经过整理；`segments` 数值与原文件逐项相等。

```bash
# 在仓库根目录执行
python3 scripts/validate_public_trajectories.py
```

这是离线研究输入，不含设备身份、最新摆位、现场预检或单次执行授权包。不要把 `OFFLINE_REFERENCE_ONLY` 标签修改成执行许可。重新部署需走详细文档中的本站点采集、仿真和打包流程。

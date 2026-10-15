# 来源与许可证说明

本仓库是现场系统的独立源码快照，基于 `F1u0rite/meituan_challenge_ws`，保留其 MIT LICENSE 与 Copyright (c) 2026 Fluorite。包含该基础上的现场修改及最新 Web UI overlay。

根许可证不能替代各组件的现有声明：部分 ROS 包声明 Apache-2.0，部分 C++ 包声明 MIT，`mtc_tool/package.xml` 保留原有 `Proprietary` 声明；整理时没有改写为统一许可证。复用或再分发时请逐组件核对原声明。

AUBO 模型、网格及上游资料的来源见 `src/mtc_simulation/assets/provenance.json`、`aubo_source_manifest.json` 和 `upstream_aubo/`；Web UI 来源见包内 `provenance.json`。CAD 与场景资产保留原来源记录。

`hardware/sdk_helpers/` 是现场播放器配套的自编 Python 封装；AUBO SDK 本体通过独立依赖安装，不随仓库重新分发。第三方库依赖由各自许可证覆盖。

本次快照整理了路径、设备地址占位、入口和文档，详见 `docs/release/source_manifest.json`。未包含凭据、原始现场录制或私有配置。

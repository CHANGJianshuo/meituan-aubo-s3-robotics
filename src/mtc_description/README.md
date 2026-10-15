# mtc_description —— 机器人/工装描述（迁移占位）

本目录是**迁移占位**，不是已迁移完成的功能包。

## 为什么是空的

本包应从旧工程 `/home/operator/meituan_challenge/src/mtc_description` 复制而来（含
`urdf/`、`s3_latch.urdf` 等）。该路径在本机**不存在**，取证见
`docs/migration/SOURCE_AUDIT.md` §4。

## 刻意不做的三件事

1. **没有 `package.xml` / `CMakeLists.txt`**：避免空包被当作已完成迁移。
2. **没有臆造 URDF / 网格**：设计文档 V0.1 §11 要求“拆工具配置：S3+V1 / S3+V2”，
   而这需要**真实 TCP、惯性、碰撞网格与 CAD**，不可凭文档描述虚构模型。
3. **没有把仿真 Ground Truth 包装成真实感知能力**。

## 补齐后如何接入

1. 只读审计并复制 URDF / mesh / launch，记录哈希；
2. 按 V1/V2 拆分工具配置；
3. 用实物标定值替换 TCP、质量、重心、碰撞参数（标定项见 `docs/architecture/TF_CONVENTIONS.md` §3）；
4. 确认腕部相机在 URDF 中挂接在运动链上而非 `world` 静态分支（`TF_CONVENTIONS.md` §1）。

## 当前状态

**迁移状态：NOT RUN**（源缺失）。

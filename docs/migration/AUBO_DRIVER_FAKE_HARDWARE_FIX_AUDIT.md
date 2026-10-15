# 官方 AUBO 驱动 Fake Hardware Xacro 本地修复审计

| 项目 | 值 |
|---|---|
| 审计时间 | 2026-10-11（Asia/Hong_Kong） |
| 审计对象 | `/home/robot/aubo_driver_validation_ws/src/aubo_ros2_driver` |
| 审计方式 | **只读**：`git status` / `git diff` / `git show` / `git submodule status` / 文件读取。未修改、未提交、未编译、未重新配置控制器 |
| 审计者 | DSH 代理（AUBO S3 C++ 迁移第一阶段） |
| 驱动是否被本次操作修改 | **否**。本阶段对该工作空间**零写入** |

---

## 1. 结论摘要

官方 `AuboRobot/aubo_ros2_driver` 的 `jazzy` 分支存在一处**上游缺陷**：`ros2_control` 的
`<hardware>` 段被整段注释，且仍使用 Foxy/Humble 时代的 `fake_components/GenericSystem`
插件名与参数名。这会导致 Mock 环境下控制器无法加载。

本地修复**仅涉及 1 个已跟踪文件**：

```
aubo_description/urdf/xacro/inc/aubo.ros2_control.xacro   (+7 / -7)
```

该修复已通过 **反向应用校验**（`git apply --check --reverse` 成功），证明归档补丁与当前
工作树内容逐字节一致，可复现。

此外，子模块工作区中存在 **154 个未跟踪的 `urdf/aubo_*.urdf.xacro`**，经取证为
**官方 launch 文件的运行时生成产物**，不是人工新增文件（见 §5）。

---

## 2. 源码版本指纹

完整指纹见 [`patches/driver_version_fingerprint.txt`](patches/driver_version_fingerprint.txt)。

### 2.1 父仓库 `aubo_ros2_driver`

| 项 | 值 |
|---|---|
| remote | `https://github.com/AuboRobot/aubo_ros2_driver.git` |
| 分支 | `jazzy` |
| HEAD | `53aab83` — *Merge pull request #21 from HysteriaC/Jazzy* |
| 工作区状态 | **干净**（`git status --short` 对自身文件无输出） |
| 相关提交 | `672c2c2` — *feat: 适配 ROS2 Jazzy*（2026-03-17） |

### 2.2 子模块 `aubo_description`

| 项 | 值 |
|---|---|
| remote | `https://github.com/AuboRobot/aubo_description.git` |
| 状态 | detached HEAD（子模块固定提交） |
| pin | `47fa5e02fa873f27f7e812d31f31e3f4cf5e56b1` — *Merge pull request #3 from HysteriaC/main* |
| 工作区状态 | **脏**：1 个 tracked 文件被修改；154 个 untracked 生成产物 |

父仓库视角：

```
$ git -C .../aubo_ros2_driver status --short --ignore-submodules=none
 m aubo_description
```

小写 `m` = 子模块 **HEAD 未变**、仅工作区内容被修改（区别于大写 `M` 表示 HEAD 已切换）。
因此修复不改变子模块 pin，可安全复现。

---

## 3. 修复内容（逐行）

补丁全文：[`patches/aubo_description_fake_hardware_jazzy.patch`](patches/aubo_description_fake_hardware_jazzy.patch)

```diff
--- a/urdf/xacro/inc/aubo.ros2_control.xacro
+++ b/urdf/xacro/inc/aubo.ros2_control.xacro
@@ -5,20 +5,20 @@
     <ros2_control name="${name}" type="system">
       <hardware>
         <!-- ros2_control simulation -->
-        <!-- xacro:if value="${use_fake_hardware}">
-          <plugin>fake_components/GenericSystem</plugin>
-          <param name="fake_sensor_commands">false</param>
-          <param name="state_following_offset">0.0</param>
-        </xacro:if -->
+        <xacro:if value="${use_fake_hardware}">
+          <plugin>mock_components/GenericSystem</plugin>
+          <param name="mock_sensor_commands">false</param>
+          <param name="position_state_following_offset">0.0</param>
+        </xacro:if>
         <!-- physical hardware or RobotStudio simulation -->
-        <!-- xacro:unless value="${use_fake_hardware}" -->
+        <xacro:unless value="${use_fake_hardware}">
           <plugin>aubo_driver/AuboHardwareInterface</plugin>
           <param name="robot_ip">${robot_ip}</param>
           ...
-        <!-- /xacro:unless -->
+        </xacro:unless>
       </hardware>
```

### 3.1 修复项分解

| # | 变更 | 原因 | 类别 |
|---|---|---|---|
| F1 | 取消注释 `<xacro:if>` / `<xacro:unless>` 两段 | 上游把整段注释后，`<hardware>` 内**没有 `<plugin>` 元素**，`ros2_control` 无法实例化硬件组件 | **阻塞性缺陷** |
| F2 | `fake_components/GenericSystem` → `mock_components/GenericSystem` | Jazzy 中 `fake_components` 已重命名为 `mock_components`（Humble 起引入，Jazzy 移除旧名） | **阻塞性缺陷** |
| F3 | `fake_sensor_commands` → `mock_sensor_commands` | 同上，Jazzy 参数重命名 | 兼容性 |
| F4 | `state_following_offset` → `position_state_following_offset` | 同上，Jazzy 参数重命名 | 兼容性 |

### 3.2 修复的语义边界（重要）

- 修复**只影响 `use_fake_hardware=true` 分支**的实际生效路径；`unless` 分支解注释后
  使 `use_fake_hardware=false` 时能正常加载 `aubo_driver/AuboHardwareInterface`。
- **本修复未使实机路径获得任何验证**。所有已完成的验证（200 Hz 控制器、0.05 rad 关节轨迹、
  MoveIt Plan→Execute）均在 `mock_components/GenericSystem` 下完成，属 Mock 环境结论。
- 物理硬件分支（`aubo_driver/AuboHardwareInterface` + `robot_ip`）在本阶段**从未运行**，
  其状态为 **NOT RUN**。

---

## 4. `xacro:unless` 解注释的影响评估

解注释 `unless` 分支使硬件插件选择由 `use_fake_hardware` 参数**真正决定**，而非"两段都注释、
恒无插件"。风险评估：

| 风险 | 评估 | 缓解 |
|---|---|---|
| 误传 `use_fake_hardware=false` 导致尝试连实机 | 存在 | 本阶段所有启动均显式传 `use_fake_hardware:=true`；且**本阶段不启动任何 AUBO launch** |
| 实机分支参数缺失（`robot_ip` 等） | 未验证 | 标注 **NOT RUN**，不纳入第一阶段结论 |

> 本阶段**不修改**该文件，不重新配置控制器，不启动该工作空间的任何 launch。

---

## 5. 154 个未跟踪 `urdf/aubo_*.urdf.xacro` 的来源取证

### 5.1 现象

```
$ git -C aubo_description status --short
 M urdf/xacro/inc/aubo.ros2_control.xacro
?? urdf/aubo_S3.urdf.xacro
?? urdf/aubo_i5.urdf.xacro
... （共 154 个）
```

### 5.2 取证结论：**官方 launch 的运行时生成产物**

生成逻辑位于父仓库**上游提交** `672c2c2`（*feat: 适配 ROS2 Jazzy*）中的
`aubo_ros2_driver/launch/aubo_control.launch.py`：

```python
def _ensure_description_xacro(description_share: str, aubo_type: str) -> None:
    urdf_dir = Path(description_share) / "urdf"
    target = urdf_dir / f"{aubo_type}.urdf.xacro"
    source = urdf_dir / f"{aubo_type}.urdf"
    if target.exists():
        return
    ...
    lines = source.read_text(encoding="utf-8").splitlines(keepends=True)
    if len(lines) >= 2 and "xmlns:xacro" not in lines[1]:
        lines[1] = lines[1].replace(">", ' xmlns:xacro="http://wiki.ros.org/xacro">', 1)
    lines = [line for line in lines if "<property" not in line]
    target.write_text("".join(lines), encoding="utf-8")
```

验证证据：

| 证据 | 结果 |
|---|---|
| 该函数是否属本地修改 | **否**。`git diff --exit-code -- aubo_ros2_driver/launch/aubo_control.launch.py` → UNCHANGED |
| 该函数是否存在于上游 HEAD | **是**。`git show HEAD:...launch.py \| grep -c _ensure_description_xacro` → 2 |
| 归属提交 | `672c2c2`（上游 `feat: 适配 ROS2 Jazzy`） |
| 生成特征是否吻合 | **吻合**。`diff urdf/aubo_S3.urdf urdf/aubo_S3.urdf.xacro` 仅两处差异：① 第 2 行注入 `xmlns:xacro`；② 删除 6 行 `<property .../>`，与函数逻辑逐行一致 |
| 文件时间戳 | 生成物 `Oct 11 11:42`；修复文件 `Oct 11 12:09` —— 生成先于修复，符合"先跑 launch，后改 xacro"的实际历程 |

**因此这 154 个文件不是人工编写的源码，而是首次运行 launch 时自动展开的 URDF 副本**，
删除后会在下次 launch 时重新生成，不影响可复现性。完整清单见
[`patches/generated_urdf_xacro_untracked.list`](patches/generated_urdf_xacro_untracked.list)。

> 注意：`_ensure_description_xacro` 生成的 `aubo_S3.urdf.xacro` **仅含机器人模型**，
> 不含 `ros2_control` 段。`ros2_control` 由 `aubo_ros2.xacro` 通过
> `aubo.ros2_control.xacro` 宏注入（见 §6）。

---

## 6. URDF / ros2_control 组装链路（AUBO S3）

```
aubo_ros2.xacro                              (urdf/xacro/inc/)
  ├─ <xacro:arg name="use_fake_hardware" default="true"/>
  ├─ <xacro:arg name="robot_ip"          default="None"/>
  ├─ <xacro:arg name="aubo_type"         default="None"/>
  ├─ include aubo_ros2_macro.xacro   →  <xacro:aubo_robot aubo_type=.../>   机器人运动学/几何
  └─ include aubo.ros2_control.xacro →  <xacro:aubo_ros2_control .../>      ros2_control 硬件段
        └─ 本审计修复点：mock_components/GenericSystem vs aubo_driver/AuboHardwareInterface
```

关节名固定为：`shoulder_joint`、`upperArm_joint`、`foreArm_joint`、`wrist1_joint`、
`wrist2_joint`、`wrist3_joint`。该命名在 `mtc_motion_validation` 的
`ValidationConfig.expected_joint_names` 中被引用，C++ 迁移必须保持同一命名与顺序。

---

## 7. 可复现性归档清单

全部归档于 `docs/migration/patches/`：

| 文件 | 用途 |
|---|---|
| [`driver_version_fingerprint.txt`](patches/driver_version_fingerprint.txt) | 父仓库 + 子模块 remote/branch/commit/subject/status 全量指纹 |
| [`aubo_description_fake_hardware_jazzy.patch`](patches/aubo_description_fake_hardware_jazzy.patch) | 修复补丁（仅 1 文件，+7/-7） |
| [`aubo.ros2_control.xacro.upstream`](patches/aubo.ros2_control.xacro.upstream) | 上游原始版本（`git show HEAD:` 导出） |
| [`aubo.ros2_control.xacro.locally_fixed`](patches/aubo.ros2_control.xacro.locally_fixed) | 本地修复后版本 |
| [`generated_urdf_xacro_untracked.list`](patches/generated_urdf_xacro_untracked.list) | 154 个运行时生成产物清单 |

### 7.1 复现步骤（供后续审计）

```bash
# 1) 确认驱动工作空间处于审计状态
cd /home/robot/aubo_driver_validation_ws/src/aubo_ros2_driver
git rev-parse HEAD                      # 期望 53aab83...
git submodule status                    # 期望 47fa5e02... aubo_description

# 2) 校验补丁与工作树一致（反向应用必须成功）
git -C aubo_description apply --check --reverse \
  /home/robot/meituan_challenge_ws/docs/migration/patches/aubo_description_fake_hardware_jazzy.patch

# 3) 校验文件内容逐字节一致
diff aubo_description/urdf/xacro/inc/aubo.ros2_control.xacro \
  /home/robot/meituan_challenge_ws/docs/migration/patches/aubo.ros2_control.xacro.locally_fixed
# 期望无输出
```

### 7.2 恢复上游原始状态的步骤（**如需回滚，须先获批准**）

> 本阶段**不执行**此操作，仅记录方法。

```bash
git -C aubo_description checkout -- urdf/xacro/inc/aubo.ros2_control.xacro
```

---

## 8. 明确的状态声明（PASS / FAIL / NOT RUN）

| 项目 | 状态 | 说明 |
|---|---|---|
| 驱动源码版本与差异已审计并归档 | **PASS** | 补丁反向应用校验通过；版本指纹已落盘 |
| 154 个未跟踪文件来源已定性 | **PASS** | 确认为官方 launch 运行时生成，非人工修改 |
| 本阶段对官方驱动工作空间零写入 | **PASS** | 仅执行只读 git 命令与文件读取 |
| Mock 环境（`mock_components/GenericSystem`）控制器与轨迹 | **NOT RUN（本阶段）** | 由上一阶段完成，本阶段未复跑、未独立验证；本节不背书其实机含义 |
| 实机硬件分支（`aubo_driver/AuboHardwareInterface`） | **NOT RUN** | 从未启动；无 `robot_ip` 连接；无安全设置变更 |
| 实机运动能力 | **NOT RUN / 未验证** | Mock 验证不代表实机执行能力 |

---

## 9. 对第一阶段的约束（本审计导出的硬规则）

1. `aubo_driver_validation_ws` **只读**，不修改驱动、不重新配置控制器、不启动其 launch。
2. C++ 迁移中的关节命名与顺序必须与 §6 一致（`shoulder_joint` … `wrist3_joint`）。
3. Mock 结论不得被表述为实机能力；所有实机项一律标注 **NOT RUN**。
4. 任何涉及 `use_fake_hardware=false` 的路径在本阶段都不得被执行或声称验证。

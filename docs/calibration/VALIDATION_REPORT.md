# 离线验收记录

2026-10-12，Windows / Python 3.11.9。全部证据由 `python -B scripts/calibration/run_offline_validation.py` 重建：命令、返回码、预期、解析后的 JSON 与生成时的 git 状态见 [results.json](validation/results.json)，逐项测试输出见 [unittest.txt](validation/unittest.txt)。该脚本只有在每项都符合预期时才 exit 0。

| 验证项 | 结果 | 证据 / 解释 |
| --- | --- | --- |
| Python 离线测试 | PASS，54/54，无跳过 | 完整位姿链、手算非共轴矩阵、非交换旋转、TCP 逆补偿、腕部随动、逆查找、异常拒绝、CLI、名义安装几何及模型/日志不一致检测 |
| 常提供的 fake 模型 / TF 日志 | PASS，仅离线材料一致性 | [fake_model_audit.json](validation/fake_model_audit.json)：独立 4×4 FK、SRDF、EE +90°、三份日志；本机未重新运行 ROS，未证明真实标定 |
| 合成 Pose/TF case | PASS，exit 0 | [synthetic.json](validation/synthetic.json)：位置约 [1,1.1,0.05]，完整姿态与 stamp 保留，simulation_only=true、motion_authorized=false |
| UNKNOWN 相机安装外参 | EXPECTED REJECTION，exit 2 | [unknown_extrinsic.json](validation/unknown_extrinsic.json)：UNKNOWN_CALIBRATION，不代入单位变换 |
| 支架设计推导 | PASS，exit 0；ESTIMATED | [nominal_mount.json](validation/nominal_mount.json)：D435i，F→depth optical [0.0825,0,0.0358] m；输出 `assumptions` 逐项标出假设与数据手册来源；不宣称 deployment_ready |
| 设计 JSON 当作观测 case | EXPECTED REJECTION，exit 2 | [design_not_real_case.json](validation/design_not_real_case.json)：INVALID_CONFIG。仅说明 schema 不兼容；状态隔离由单元测试覆盖 |
| 站点清单当作观测 case | EXPECTED REJECTION，exit 2 | results.json `site_inventory_not_a_case`：INVALID_POSE，没有观测，不能形成目标 |
| 生成的反例 | EXPECTED REJECTION，两个 CLI 均 exit 2 | [generated_negative_inputs.json](validation/generated_negative_inputs.json)：10**400 坐标 → INVALID_VECTOR；20 万层嵌套 JSON → INVALID_INPUT；设计中把 mm 写成 m → IMPLAUSIBLE_DESIGN；所有输出 motion_authorized=false |
| JSON / CSV 列数 / 本地链接 / 清单 null 纪律 | PASS | 由脚本检查：交付 JSON 可解析、CSV 列数一致、交接与 calibration 文档本地链接存在、site_inventory 的 UNKNOWN 边与阈值仍为 null |
| ROS/Jazzy 节点、TF 监听和集成 | NOT RUN | 本机离线工具没有 ROS 依赖；现场只读命令已提供但未执行 |
| 手眼求解、实物外参、V1 TCP、台面/T0 误差 | NOT RUN | 缺现场数据；清单仍为 UNKNOWN；未实现或调用手眼求解器 |
| 自动规划拒绝规则接入 | NOT IMPLEMENTED HERE | 诊断工具输出拒绝原因；实际适配器接入由对应成员完成 |

## 过程记录

- 首轮（Codex）：基础校验模块未实现时，门禁测试失败 1 项、28 项跳过；安装推导脚本未实现时，门禁失败 1 项、4 项跳过；随后 34 项通过。跳过的用例不算已观察到的行为失败。当时的超大整数问题按原审查意见修正。
- 独立复核（Claude，报告：`D:\RA\美团比赛\Claude复核\Claude_independent_review_20261011_1957\REVIEW_REPORT.md`）：外参代数经独立矩阵与 NumPy 交叉核算正确；指出 17.5 mm 的基准是 1/4-20 中心线而非外壳中心、M3 孔位无官方尺寸，以及若干 CLI/文档小问题。
- 修正轮：先新增 14 项测试并运行，观察到 13 项失败、2 项错误（深嵌套 JSON 退出码、OUTPUT_ERROR 字段、schema_version 类型、int64 时间戳、camera_model 与 D435f/D435if 偏移、假设清单、尺度保护）；实现后 48/48 通过。复核者对修正后代码重跑独立外参探针、39 个契约边界用例与 22 个变异体，变异体全部被测试发现。
- 本轮重验：Claude 优化的 48 项先全部复跑通过；只读独立复核另查 11 个 CLI 异常输入均拒绝。同步规划 main 后新增 fake 材料核验，功能存在门禁先失败 1 项、5 项跳过，实现后全部 6 项通过，总计 54 项。删 EE 旋转、错平移符号、缺日志样本及错 SRDF tip 均被检测。共享 PDF 已渲染目视核验；不将图中固定 TF 的时间 0 当作观测时间。

这些验证证明离线数学和输入契约按测试样例工作，不证明机器人运动正确、任务达到比赛精度或团队适配器已消费校验结果。MEASURED evidence 在工具中是引用检查；证据真实性、误差和安装稳定性由现场复核。

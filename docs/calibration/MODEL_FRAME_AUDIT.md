# S3 模型与末端 frame 核验

2026-10-11 初版；2026-10-12 补充 fake 展开模型及 TF 日志核验。真实运行 TF、实物安装未核验。

最新 main 的共享 URDF/SRDF/TF 与官方链一致；独立 FK 和全部有效日志样本的复算见 [FAKE_RUNTIME_AUDIT.md](FAKE_RUNTIME_AUDIT.md)。常的 fake 运行报告已确认 Planning Frame=`world`、EE=`ee_link`；下述初版来源仍保留固定版本供溯源，现场需核对真实部署，fake 证据不提升真实测量状态。

团队 `main` 固定快照 `1d940f7c5f12561fed6641457c0b4b630a282b05` 的驱动文档给出：

- 驱动：`AuboRobot/aubo_ros2_driver`，Jazzy，`53aab83735236f0c262ecb11e04ef2600241d9d9`。
- 模型：`AuboRobot/aubo_description`，`47fa5e02fa873f27f7e812d31f31e3f4cf5e56b1`。
- SRDF：group `manipulator`，chain `base_link → ee_link`。这是团队文档记录；现场需要核对实际加载模型和 `getPlanningFrame()`/EE link。

本次另外直接读取该固定提交的官方源码，获得：

| 文件 | SHA256 |
| --- | --- |
| `urdf/aubo_S3.urdf` | `2ad1d85b3ab53b82f1944a373e3e41b36ca2535e0edf5b70c9c32641679e1924` |
| `urdf/xacro/inc/aubo_ros2_macro.xacro` | `8bbf8f5e127650dd79760f3b2d3025d734fae4b6cfaaee3a923e923022e7a3b7` |

注意来源层级：`aubo_ros2_macro.xacro:5` include 的是 `$(find aubo_description)/urdf/${aubo_type}.urdf.xacro`。固定提交的源码树里只有 `urdf/aubo_S3.urdf`，没有 `aubo_S3.urdf.xacro`；团队驱动文档说明后者由构建/Launch 生成。下面的链条来自 `aubo_S3.urdf`，`ee_fixed_joint` 来自宏本身；运行时加载的模型需保存 `robot_description` 哈希与之对比后，才能认定一致。

模型连杆链：`world → base_link → shoulder_Link → upperArm_Link → foreArm_Link → wrist1_Link → wrist2_Link → wrist3_Link`。六个关节顺序为 `shoulder_joint`、`upperArm_joint`、`foreArm_joint`、`wrist1_joint`、`wrist2_joint`、`wrist3_joint`。

Xacro 宏另加 `ee_fixed_joint`：parent=`wrist3_Link`，child=`ee_link`，xyz=`0 0 0`，rpy=`0 0 ${pi/2.0}`。即 `T_wrist3_ee` 的模型平移为零、绕 Z 旋转 +π/2，四元数 xyzw 为 `[0,0,sin(π/4),cos(π/4)]`。这是模型定义，不是实物相机/TCP 的标定值。记录表中把模型几何归为 ESTIMATED，并标明源码已核验；保存运行时 `tf2_echo wrist3_Link ee_link` 与 `robot_description` 读数后才可改为 MEASURED（运行时读取）。

S3 末端视觉网格 `meshes/aubo_S3/visual/link6.DAE` 的 `<unit name="centimeter" meter="1.0">` 名称写厘米但换算因子为 1，坐标单位是米；节点 `S3-JION6_1` 含 z 向 -0.05561 m 平移。按此解析，外形在 `wrist3_Link` 中约 X ±36 mm、Y -36～39.5 mm、Z -55.61～0 mm，端面在 z=0、+Z 指向工具侧。网格只支持“`wrist3_Link` 原点与 +Z 名义上在法兰端面”，不给出孔位、定位销角度或公差，因此 F 的 +X 与 `wrist3_Link` 的偏航关系仍未知。

**不能将 `wrist3_Link` 与 `ee_link` 的坐标直接互换。** “相机装法兰附近”还不能证明其外参应当以哪一个 frame 记录。应当先确定标定的末端参考系 E，再保存 `T_E_camera` 与 `T_E_TCP`；必要时通过该固定变换转到规划 EE。

此两文件没有提供你们的 D435i 安装外参、V1 舌规 TCP 或台面/工位数据。团队 TF 草案 `tool0/hook_tcp` 与配置 `tool0_v1` 也不能替代运行中的真实 frame。交付暂提议使用 `hook_tcp_v1`，最终名称需团队确认。

来源：[团队驱动文档](https://github.com/F1u0rite/meituan_challenge_ws/blob/1d940f7c5f12561fed6641457c0b4b630a282b05/docs/setup/AUBO_ROS2_DRIVER_JAZZY.md)、[官方 S3 URDF](https://github.com/AuboRobot/aubo_description/blob/47fa5e02fa873f27f7e812d31f31e3f4cf5e56b1/urdf/aubo_S3.urdf)、[官方末端宏](https://github.com/AuboRobot/aubo_description/blob/47fa5e02fa873f27f7e812d31f31e3f4cf5e56b1/urdf/xacro/inc/aubo_ros2_macro.xacro)。

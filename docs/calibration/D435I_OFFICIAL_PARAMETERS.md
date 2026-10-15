# 官方提供哪些 D435i 参数

2026-10-11 查阅官方 SDK 与 ROS Wrapper 资料。

## 1. 两类外参分别记录

| 参数 | 官方支持 | 本项目当前状态 |
| --- | --- | --- |
| 彩色/深度流内参、畸变模型 | SDK `get_intrinsics()` 或实际流 `CameraInfo` | UNKNOWN：未读取当前设备/分辨率 |
| 深度、彩色等内部传感器间外参 | SDK `get_extrinsics_to()`；ROS Wrapper 的相应内部 TF/Extrinsics | UNKNOWN：可从当前设备读取，不能照抄官网示例数值 |
| 相机光学系轴向 | X 右、Y 下、Z 前；ROS 相机机体系 X 前、Y 左、Z 上 | 约定已知，实际 frame 名与内部 TF 待读取 |
| 法兰/末端参考系到 D435i | 与本团队支架和安装有关；官方内部传感器外参不能提供这段关系 | UNKNOWN：手眼标定，或 CAD/测量给 ESTIMATED 后复核 |
| V1 工具 TCP | 与舌规装配和工具工作点有关 | UNKNOWN：工具测量/标定 |

方向与布局（已对照固定版本源码）：

- SDK `rs2_extrinsics` 与 ROS `realsense2_camera_msgs/Extrinsics` 的 `rotation` 都是 9 元素 **column-major** 矩阵，平移单位 m（librealsense `include/librealsense2/h/rs_sensor.h:113-114`，提交 `e15c5d6bb1563e778d116f682aeefffbae2daedc`；realsense-ros README “Extrinsics from sensor A to sensor B”）。
- `rs2_transform_point_to_point` 计算 `p_to = R·p_from + t`（librealsense `src/rs.cpp:4475-4480`）。因此 `from=depth, to=color` 的外参，即 ROS 话题 `/extrinsics/depth_to_color`，在本项目约定中是 **`T_color_depth`**：把深度系点映射到彩色系，`t` 是深度原点在彩色系中的位置。需要 `T_depth_color` 时取逆。
- 九元素到矩阵：`R = [[r0,r3,r6],[r1,r4,r7],[r2,r5,r8]]`；NumPy 用 `np.array(r).reshape(3, 3, order='F')`。按行 reshape 得到的是转置（即反向旋转），不会报错，只会悄悄错。

本文不提供某台其他 D435i 的外参数值；README 中的示例数值只用于说明格式。

官方资料存在年代不同的 IMU 内参描述，本任务不据此开展 IMU 标定；当前任务只需颜色/深度定位与末端安装外参。

## 2. 明天优先读取现有设备参数

先由视觉/集成人员启动已确认版本的 D435i ROS 节点。本交付不启动相机、不更改设备 EEPROM、不重新烧写标定。

```bash
ros2 topic list -t
ros2 node list
```

在实际话题列表中选择彩色/深度 `camera_info`、内部 extrinsics 和图像话题。官方常见例子为 `/camera/camera/color/camera_info`、`/camera/camera/depth/camera_info`、`/camera/camera/extrinsics/depth_to_color`，**这些名称并非本项目已核验值**。设定变量为实际值后执行只读导出：

```bash
# 将变量值换成上一步实际发现的话题，保留证据目录。
export COLOR_INFO_TOPIC=/camera/camera/color/camera_info
export DEPTH_INFO_TOPIC=/camera/camera/depth/camera_info
export INTERNAL_EXTRINSICS_TOPIC=/camera/camera/extrinsics/depth_to_color
mkdir -p runtime_logs/calibration/device
ros2 topic echo "$COLOR_INFO_TOPIC" --once > runtime_logs/calibration/device/color_camera_info.yaml
ros2 topic echo "$DEPTH_INFO_TOPIC" --once > runtime_logs/calibration/device/depth_camera_info.yaml
ros2 topic info "$INTERNAL_EXTRINSICS_TOPIC" --verbose
# transient_local/reliable 应与上一步实际发布者 QoS 匹配；不要盲目套用。
ros2 topic echo "$INTERNAL_EXTRINSICS_TOPIC" --once --qos-durability transient_local --qos-reliability reliable > runtime_logs/calibration/device/depth_to_color.yaml
```

同时只读保存内部 TF（`tf2_echo A B` 打印 `T_A_B`，即 B 在 A 中的位姿；它的 “source/target” 参数名与 `lookup_transform(target, source)` 相反，见 ros2/geometry2 jazzy `tf2_ros/src/tf2_echo.cpp`）。帧名按实际 `camera_name`/命名空间替换：

```bash
ros2 run tf2_ros tf2_echo camera_link camera_depth_optical_frame   # 期望 t≈0，q≈[-0.5,0.5,-0.5,0.5]
ros2 run tf2_ros tf2_echo camera_link camera_color_optical_frame   # t 以本机为准（彩色约在 +Y 1.5 cm 量级）
```

没有消息时保留 NOT RUN/FAIL 原因，不空等认定成功。CameraInfo 需核对分辨率、K/D/R/P、distortion_model、header.frame_id，并记录设备序列号、固件、SDK/Wrapper 版本和视觉是否将深度对齐彩色。使用校正图、对齐深度或 resize 图时必须采用相应投影/内参处理，不能套其他流的矩阵。

可以先复用设备参数并用真实测量点检查质量；只有验证显示问题或输入处理不匹配时，再决定是否额外标定相机内参。当前没有数据，不能宣称已完成该质量验证。

## 3. 官方来源

- [SDK 投影与内外参](https://github.com/realsenseai/librealsense/wiki/Projection-in-RealSense-SDK-2.0#extrinsic-camera-parameters)：流间外参、`get_extrinsics_to()`、m 单位。
- [ROS Wrapper](https://github.com/realsenseai/realsense-ros#extrinsics-from-sensor-a-to-sensor-b)：内部外参话题及 column-major。
- [坐标轴约定](https://github.com/realsenseai/realsense-ros#ros2robot-vs-opticalcamera-coordination-systems)：光学系和 ROS 系不同。
- [D435i SDK 说明](https://github.com/realsenseai/librealsense/blob/master/doc/d435i.md)：内部 IMU/深度坐标关系。

读取到设备内参/内部外参后，该项可标 MEASURED，来源注明 DEVICE_READBACK；它不意味着法兰至相机的手眼外参也完成。

支架可重新制作/调整后的布局提案、前玻璃/左 IR 原点区别及设计外参推导见 [S3_D435I_MOUNT_DESIGN.md](S3_D435I_MOUNT_DESIGN.md)。官方 PDF 已下载并渲染核对图页；45 mm 安装孔距可以用于支架规划，实物到法兰的关系仍由具体安装定义。

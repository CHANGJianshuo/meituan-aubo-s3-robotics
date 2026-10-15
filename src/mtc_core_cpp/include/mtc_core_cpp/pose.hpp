// Copyright 2026 meituan_challenge_ws contributors
//
// 6D 位姿保真类型。
//
// 迁移动机（见 docs/migration/INTERFACE_COMPAT_AUDIT_CPP.md §2）：
//   Python 侧 pick_place_fsm.PoseEstimate 把位姿退化为 Tuple[float,float,float]，
//   丢弃了 orientation、frame_id 与时间戳；pick_place_server.py 读取
//   BatteryDetection.msg 的 PoseStamped 时也只取 position。
//
// 需求 5 明确要求："必须保留完整 6D Pose、frame_id 和 TF 信息，
// 不得退化成只有 XYZ 的目标位置。"
//
// 因此 C++ 侧 Pose6D 强制携带 position + orientation + frame_id + stamp，
//   且**不提供**任何只含 XYZ 的构造函数重载，从类型层面阻止再次退化。

#ifndef MTC_CORE_CPP__POSE_HPP_
#define MTC_CORE_CPP__POSE_HPP_

#include <array>
#include <cmath>
#include <cstdint>
#include <string>

namespace mtc
{
namespace core
{

struct Vector3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  bool operator==(const Vector3 & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
  bool operator!=(const Vector3 & other) const {return !(*this == other);}
};

/// 单位四元数 (x, y, z, w)，与 geometry_msgs/Quaternion 字段顺序一致。
struct Quaternion
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;

  bool operator==(const Quaternion & other) const
  {
    return x == other.x && y == other.y && z == other.z && w == other.w;
  }
  bool operator!=(const Quaternion & other) const {return !(*this == other);}

  double norm() const {return std::sqrt(x * x + y * y + z * z + w * w);}

  /// 是否为可用的归一化四元数（容忍 1e-3 数值误差）。
  bool is_normalized(double tolerance = 1e-3) const
  {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(w)) {
      return false;
    }
    return std::fabs(norm() - 1.0) <= tolerance;
  }
};

/// 完整 6D 位姿：等价于 geometry_msgs/PoseStamped 的语义承载。
///
/// frame_id 与 stamp_s 是一等公民，不可缺省为"空/0 即无所谓"：
/// 任何跨坐标系使用都必须先核对 frame_id（见 docs/architecture/TF_CONVENTIONS.md）。
struct Pose6D
{
  Vector3 position;
  Quaternion orientation;
  std::string frame_id;   ///< TF 父坐标系名；空字符串表示未标注，属无效位姿
  double stamp_s = 0.0;   ///< 观测时刻（秒，与 Clock 同一时基）
  bool valid = false;     ///< 显式有效性；false 时禁止据此生成运动目标

  /// 有效性要求：显式 valid、frame_id 非空、且四元数已归一化。
  bool is_usable() const
  {
    return valid && !frame_id.empty() && orientation.is_normalized() &&
           std::isfinite(position.x) && std::isfinite(position.y) &&
           std::isfinite(position.z) && std::isfinite(stamp_s) && stamp_s >= 0.0;
  }

  bool operator==(const Pose6D & other) const
  {
    return position == other.position && orientation == other.orientation &&
           frame_id == other.frame_id && stamp_s == other.stamp_s && valid == other.valid;
  }
  bool operator!=(const Pose6D & other) const {return !(*this == other);}
};

/// 一次目标位姿观测（迁移自 Python PoseEstimate，并补齐 6D 与坐标系）。
struct PoseEstimate
{
  std::string object_id;
  int color = 0;                           ///< BatteryDetection.msg 颜色常量
  bool has_ring_pose_value = false;        ///< ring_pose 是否被观测到
  Pose6D ring_pose;                        ///< 提环局部抓取参考坐标系
  bool has_body_pose_value = false;        ///< body_pose 是否被观测到
  Pose6D body_pose;                        ///< 电池主体中心
  float confidence = 0.0f;
  double stamp_s = 0.0;
  bool ring_pose_valid = true;             ///< BatteryDetection.msg ring_pose_valid
  bool at_target = false;                  ///< 是否位于目标区域内（VERIFY_PLACED 用）
  bool settled = true;                     ///< 是否已静止（VERIFY_PLACED 用）
  /// 视觉是否确认"物体随工具一起运动"（VERIFY_ATTACHED 的独立证据之一）
  bool followed = false;

  /// 提环位姿是否可用作运动目标（观测存在 + 标注有效 + 本身可用）。
  bool has_ring_pose() const {return has_ring_pose_value && ring_pose_valid && ring_pose.is_usable();}
  bool has_body_pose() const {return has_body_pose_value && body_pose.is_usable();}
};

}  // namespace core
}  // namespace mtc

#endif  // MTC_CORE_CPP__POSE_HPP_

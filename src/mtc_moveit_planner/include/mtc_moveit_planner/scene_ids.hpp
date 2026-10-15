// Copyright 2026 meituan_challenge_ws contributors
//
// 规划场景对象 ID 隔离（纯 C++）。
//
// move_group 的 Planning Scene 是全局共享的：感知、工具、集成等模块也可能往里写碰撞体。
// 本模块写入的所有对象（静态障碍物、附着的电池载荷）统一加前缀 kSceneIdNamespace，
// 并且增删时只操作带该前缀的对象，从而：
//   * 不会因为同名（如 "table"、"battery"）覆盖或删除其他模块的对象；
//   * 规划进程重启后，能清掉自己上次遗留的对象，而不必记住旧 ID。
// 配置里写的是"本地 ID"（如 table），不允许自带前缀或 '/'，也不允许重复。

#ifndef MTC_MOVEIT_PLANNER__SCENE_IDS_HPP_
#define MTC_MOVEIT_PLANNER__SCENE_IDS_HPP_

#include <string>
#include <vector>

namespace mtc
{
namespace moveit_planner
{

/// 本模块在 Planning Scene 中的 ID 命名空间。
constexpr const char kSceneIdNamespace[] = "mtc_moveit_planner/";

/// 本地 ID 只允许 [A-Za-z0-9_.-]，非空且不超过 64 字符。
bool is_valid_local_scene_id(const std::string & local_id);

/// 本地 ID → 场景中的完整 ID（加命名空间前缀）。调用前应先校验本地 ID。
std::string scoped_scene_id(const std::string & local_id);

/// 场景中的 ID 是否属于本模块（带本模块前缀）。
bool is_owned_scene_id(const std::string & scene_id);

/// 校验一组本地 ID：全部合法且互不重复；不通过时返回原因，通过返回空串。
std::string validate_local_scene_ids(const std::vector<std::string> & local_ids);

/// 从场景已有对象中挑出"本模块拥有、但不在 keep 中"的 ID（需要移除的遗留对象）。
/// 不属于本模块的 ID 永远不会出现在结果中。
std::vector<std::string> stale_owned_scene_ids(
  const std::vector<std::string> & known_scene_ids,
  const std::vector<std::string> & keep_scene_ids);

}  // namespace moveit_planner
}  // namespace mtc

#endif  // MTC_MOVEIT_PLANNER__SCENE_IDS_HPP_

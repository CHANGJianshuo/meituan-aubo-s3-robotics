// Copyright 2026 meituan_challenge_ws contributors

#include "mtc_moveit_planner/scene_ids.hpp"

#include <algorithm>
#include <set>

namespace mtc
{
namespace moveit_planner
{

bool is_valid_local_scene_id(const std::string & local_id)
{
  if (local_id.empty() || local_id.size() > 64) {
    return false;
  }
  return std::all_of(local_id.begin(), local_id.end(), [](char c) {
             return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
             (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.';
           });
}

std::string scoped_scene_id(const std::string & local_id)
{
  return std::string(kSceneIdNamespace) + local_id;
}

bool is_owned_scene_id(const std::string & scene_id)
{
  const std::string prefix(kSceneIdNamespace);
  return scene_id.size() > prefix.size() && scene_id.compare(0, prefix.size(), prefix) == 0;
}

std::string validate_local_scene_ids(const std::vector<std::string> & local_ids)
{
  std::set<std::string> seen;
  for (const auto & id : local_ids) {
    if (!is_valid_local_scene_id(id)) {
      return "invalid scene object id '" + id + "' (use 1-64 chars of [A-Za-z0-9_.-], no prefix)";
    }
    if (!seen.insert(id).second) {
      return "duplicate scene object id '" + id + "'";
    }
  }
  return "";
}

std::vector<std::string> stale_owned_scene_ids(
  const std::vector<std::string> & known_scene_ids,
  const std::vector<std::string> & keep_scene_ids)
{
  const std::set<std::string> keep(keep_scene_ids.begin(), keep_scene_ids.end());
  std::set<std::string> stale;
  for (const auto & id : known_scene_ids) {
    if (is_owned_scene_id(id) && keep.count(id) == 0) {
      stale.insert(id);
    }
  }
  return {stale.begin(), stale.end()};
}

}  // namespace moveit_planner
}  // namespace mtc

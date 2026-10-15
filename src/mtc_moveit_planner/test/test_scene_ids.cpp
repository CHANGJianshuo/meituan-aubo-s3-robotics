// Copyright 2026 meituan_challenge_ws contributors

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "mtc_moveit_planner/planning_backend.hpp"
#include "mtc_moveit_planner/scene_ids.hpp"

namespace
{

using namespace mtc::moveit_planner;  // NOLINT(build/namespaces)

TEST(SceneIds, LocalIdsAreScopedIntoPlannerNamespace)
{
  EXPECT_EQ(scoped_scene_id("table"), "mtc_moveit_planner/table");
  EXPECT_TRUE(is_owned_scene_id("mtc_moveit_planner/table"));
  // 其他模块的同名对象、前缀相似的对象、裸前缀都不属于本模块。
  EXPECT_FALSE(is_owned_scene_id("table"));
  EXPECT_FALSE(is_owned_scene_id("perception/table"));
  EXPECT_FALSE(is_owned_scene_id("mtc_moveit_planner_old/table"));
  EXPECT_FALSE(is_owned_scene_id("mtc_moveit_planner/"));
  // 载荷也在本模块命名空间内，不会和感知发布的 "battery_payload" 等对象同名。
  EXPECT_TRUE(is_owned_scene_id(AttachedPayload{}.id));
}

TEST(SceneIds, InvalidOrDuplicateLocalIdsAreRejected)
{
  EXPECT_EQ(validate_local_scene_ids({"table", "fixture_T0", "cam.mount-1"}), "");
  EXPECT_NE(validate_local_scene_ids({"table", "table"}).find("duplicate"), std::string::npos);
  EXPECT_NE(validate_local_scene_ids({""}), "");
  EXPECT_NE(validate_local_scene_ids({"mtc_moveit_planner/table"}), "");  // 不允许自带前缀
  EXPECT_NE(validate_local_scene_ids({"other/table"}), "");
  EXPECT_NE(validate_local_scene_ids({"has space"}), "");
  EXPECT_NE(validate_local_scene_ids({std::string(65, 'a')}), "");
}

TEST(SceneIds, OnlyOwnedLeftoversAreRemoved)
{
  const std::vector<std::string> known{
    "table",                            // 其他模块
    "battery_3",                        // 感知
    "mtc_moveit_planner/table",         // 本模块，本次保留
    "mtc_moveit_planner/old_fixture",   // 本模块上次遗留（如进程重启前写入）
    "mtc_moveit_planner_old/x"};        // 前缀相似但不属于本模块
  const auto stale = stale_owned_scene_ids(known, {"mtc_moveit_planner/table"});
  ASSERT_EQ(stale.size(), 1u);
  EXPECT_EQ(stale[0], "mtc_moveit_planner/old_fixture");
  EXPECT_TRUE(stale_owned_scene_ids({"table", "battery_3"}, {}).empty());
}

}  // namespace

// Copyright 2026 meituan_challenge_ws contributors
//
// 唯一执行权测试：并发运动请求必须被拒绝（需求 5）。

#include <gtest/gtest.h>

#include "mtc_motion_execution_cpp/in_flight_guard.hpp"

namespace
{
using mtc::motion::InFlightGuard;
}  // namespace

TEST(InFlightGuard, AcquireSucceedsWhenFree)
{
  InFlightGuard guard;
  EXPECT_TRUE(guard.is_free());

  const auto result = guard.acquire("cmd-1");
  EXPECT_TRUE(result.first);
  EXPECT_EQ(result.second, "acquired");
  EXPECT_FALSE(guard.is_free());
  ASSERT_TRUE(guard.in_flight().has_value());
  EXPECT_EQ(*guard.in_flight(), "cmd-1");
}

TEST(InFlightGuard, SecondAcquireIsRejectedWithInFlightId)
{
  InFlightGuard guard;
  ASSERT_TRUE(guard.acquire("cmd-1").first);

  const auto second = guard.acquire("cmd-2");
  EXPECT_FALSE(second.first);
  // 拒绝理由必须指出当前在途命令，便于定位并发冲突
  EXPECT_NE(second.second.find("cmd-1"), std::string::npos);
  EXPECT_NE(second.second.find("cmd-2"), std::string::npos);
  EXPECT_EQ(*guard.in_flight(), "cmd-1");
}

TEST(InFlightGuard, EmptyCommandIdRejected)
{
  InFlightGuard guard;
  const auto result = guard.acquire("");
  EXPECT_FALSE(result.first);
  EXPECT_EQ(result.second, "command_id 不能为空");
  EXPECT_TRUE(guard.is_free());
}

TEST(InFlightGuard, ReleaseOnlyByOwner)
{
  InFlightGuard guard;
  ASSERT_TRUE(guard.acquire("cmd-1").first);

  // 非持有者释放无效
  guard.release("cmd-other");
  EXPECT_FALSE(guard.is_free());

  guard.release("cmd-1");
  EXPECT_TRUE(guard.is_free());

  // 释放后可再次取得
  EXPECT_TRUE(guard.acquire("cmd-2").first);
}

TEST(InFlightGuard, ForceReleaseIsAvailableForManualRecovery)
{
  InFlightGuard guard;
  ASSERT_TRUE(guard.acquire("cmd-1").first);
  guard.force_release();
  EXPECT_TRUE(guard.is_free());
}

TEST(InFlightGuard, SequentialCommandsAreAllowed)
{
  InFlightGuard guard;
  for (int i = 0; i < 5; ++i) {
    const std::string id = "cmd-" + std::to_string(i);
    ASSERT_TRUE(guard.acquire(id).first);
    guard.release(id);
  }
  EXPECT_TRUE(guard.is_free());
}

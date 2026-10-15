// Copyright 2026 meituan_challenge_ws contributors
//
// 可注入时钟抽象（迁移自 Python ClockPort / FakeClock / backends.Clock）。
//
// 设计约束：状态机的超时判定必须完全依赖注入时钟，
// 不得在核心逻辑中直接调用 std::chrono::steady_clock，
// 否则单元测试无法在不睡眠的前提下验证超时与看门狗语义。

#ifndef MTC_CORE_CPP__CLOCK_HPP_
#define MTC_CORE_CPP__CLOCK_HPP_

#include <chrono>
#include <atomic>
#include <memory>

namespace mtc
{
namespace core
{

/// 时钟接口。now() 返回单调递增的秒数。
class Clock
{
public:
  virtual ~Clock() = default;
  virtual double now() const = 0;
};

using ClockPtr = std::shared_ptr<Clock>;

/// 生产用时钟：基于 std::chrono::steady_clock。
class SteadyClock : public Clock
{
public:
  double now() const override
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }
};

/// 离线测试假时钟：不随真实时间流逝，由测试/Mock 显式推进。
/// 使用者必须共享同一个实例，数值起点相同不代表同一时基；不得用于实机证据。
class ManualClock : public Clock
{
public:
  explicit ManualClock(double start_s = 0.0)
  : now_s_(start_s) {}

  double now() const override {return now_s_.load();}

  void advance(double seconds)
  {
    if (seconds > 0.0) {
      double previous = now_s_.load();
      while (!now_s_.compare_exchange_weak(previous, previous + seconds)) {}
    }
  }

  void set(double seconds) {now_s_ = seconds;}

private:
  std::atomic<double> now_s_{0.0};
};

}  // namespace core
}  // namespace mtc

#endif  // MTC_CORE_CPP__CLOCK_HPP_

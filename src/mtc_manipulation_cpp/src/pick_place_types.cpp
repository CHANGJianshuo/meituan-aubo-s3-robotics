// Copyright 2026 meituan_challenge_ws contributors

#include "mtc_manipulation_cpp/pick_place_types.hpp"

#include <algorithm>

namespace mtc
{
namespace manipulation
{

const std::vector<std::string> & allowed_target_slots()
{
  static const std::vector<std::string> kSlots = {"T0", "P1", "P2", "P3"};
  return kSlots;
}

const char * to_string(PickPlaceState state)
{
  switch (state) {
    case PickPlaceState::RESOLVE_TARGET: return "RESOLVE_TARGET";
    case PickPlaceState::PLAN_ACQUIRE: return "PLAN_ACQUIRE";
    case PickPlaceState::MOVE_PRE_ALIGN: return "MOVE_PRE_ALIGN";
    case PickPlaceState::ENGAGE: return "ENGAGE";
    case PickPlaceState::VERIFY_ENGAGEMENT: return "VERIFY_ENGAGEMENT";
    case PickPlaceState::TEST_LIFT: return "TEST_LIFT";
    case PickPlaceState::VERIFY_ATTACHED: return "VERIFY_ATTACHED";
    case PickPlaceState::LIFT: return "LIFT";
    case PickPlaceState::TRANSPORT: return "TRANSPORT";
    case PickPlaceState::MOVE_PRE_SEAT: return "MOVE_PRE_SEAT";
    case PickPlaceState::SEAT: return "SEAT";
    case PickPlaceState::VERIFY_SEATED: return "VERIFY_SEATED";
    case PickPlaceState::UNLOAD: return "UNLOAD";
    case PickPlaceState::DISENGAGE: return "DISENGAGE";
    case PickPlaceState::RETREAT: return "RETREAT";
    case PickPlaceState::VERIFY_PLACED: return "VERIFY_PLACED";
    case PickPlaceState::SUCCESS: return "SUCCESS";
    case PickPlaceState::RECOVERY: return "RECOVERY";
    case PickPlaceState::FAILED: return "FAILED";
    case PickPlaceState::FAULT: return "FAULT";
    case PickPlaceState::CANCEL_PENDING: return "CANCEL_PENDING";
    case PickPlaceState::CANCELLED: return "CANCELLED";
  }
  return "UNKNOWN";
}

const std::vector<PickPlaceState> & all_pick_place_states()
{
  static const std::vector<PickPlaceState> kStates = {
    PickPlaceState::RESOLVE_TARGET,
    PickPlaceState::PLAN_ACQUIRE,
    PickPlaceState::MOVE_PRE_ALIGN,
    PickPlaceState::ENGAGE,
    PickPlaceState::VERIFY_ENGAGEMENT,
    PickPlaceState::TEST_LIFT,
    PickPlaceState::VERIFY_ATTACHED,
    PickPlaceState::LIFT,
    PickPlaceState::TRANSPORT,
    PickPlaceState::MOVE_PRE_SEAT,
    PickPlaceState::SEAT,
    PickPlaceState::VERIFY_SEATED,
    PickPlaceState::UNLOAD,
    PickPlaceState::DISENGAGE,
    PickPlaceState::RETREAT,
    PickPlaceState::VERIFY_PLACED,
    PickPlaceState::SUCCESS,
    PickPlaceState::RECOVERY,
    PickPlaceState::FAILED,
    PickPlaceState::FAULT,
    PickPlaceState::CANCEL_PENDING,
    PickPlaceState::CANCELLED,
  };
  return kStates;
}

const std::set<PickPlaceState> & terminal_states()
{
  static const std::set<PickPlaceState> kStates = {
    PickPlaceState::SUCCESS,
    PickPlaceState::FAILED,
    PickPlaceState::FAULT,
    PickPlaceState::CANCELLED,
  };
  return kStates;
}

const std::set<PickPlaceState> & motion_in_flight_states()
{
  static const std::set<PickPlaceState> kStates = {
    PickPlaceState::MOVE_PRE_ALIGN,
    PickPlaceState::ENGAGE,
    PickPlaceState::TEST_LIFT,
    PickPlaceState::LIFT,
    PickPlaceState::TRANSPORT,
    PickPlaceState::MOVE_PRE_SEAT,
    PickPlaceState::SEAT,
    PickPlaceState::UNLOAD,
    PickPlaceState::DISENGAGE,
    PickPlaceState::RETREAT,
  };
  return kStates;
}

const std::vector<PickPlaceState> & pipeline_states()
{
  static const std::vector<PickPlaceState> kStates = {
    PickPlaceState::RESOLVE_TARGET,
    PickPlaceState::PLAN_ACQUIRE,
    PickPlaceState::MOVE_PRE_ALIGN,
    PickPlaceState::ENGAGE,
    PickPlaceState::VERIFY_ENGAGEMENT,
    PickPlaceState::TEST_LIFT,
    PickPlaceState::VERIFY_ATTACHED,
    PickPlaceState::LIFT,
    PickPlaceState::TRANSPORT,
    PickPlaceState::MOVE_PRE_SEAT,
    PickPlaceState::SEAT,
    PickPlaceState::VERIFY_SEATED,
    PickPlaceState::UNLOAD,
    PickPlaceState::DISENGAGE,
    PickPlaceState::RETREAT,
    PickPlaceState::VERIFY_PLACED,
    PickPlaceState::SUCCESS,
  };
  return kStates;
}

std::map<PickPlaceState, double> default_state_timeouts()
{
  return {
    {PickPlaceState::RESOLVE_TARGET, 2.0},
    {PickPlaceState::PLAN_ACQUIRE, 2.0},
    {PickPlaceState::MOVE_PRE_ALIGN, 30.0},
    {PickPlaceState::ENGAGE, 15.0},
    {PickPlaceState::VERIFY_ENGAGEMENT, 5.0},
    {PickPlaceState::TEST_LIFT, 15.0},
    {PickPlaceState::VERIFY_ATTACHED, 5.0},
    {PickPlaceState::LIFT, 30.0},
    {PickPlaceState::TRANSPORT, 60.0},
    {PickPlaceState::MOVE_PRE_SEAT, 30.0},
    {PickPlaceState::SEAT, 20.0},
    {PickPlaceState::VERIFY_SEATED, 5.0},
    {PickPlaceState::UNLOAD, 15.0},
    {PickPlaceState::DISENGAGE, 20.0},
    {PickPlaceState::RETREAT, 30.0},
    {PickPlaceState::VERIFY_PLACED, 30.0},
  };
}

}  // namespace manipulation
}  // namespace mtc

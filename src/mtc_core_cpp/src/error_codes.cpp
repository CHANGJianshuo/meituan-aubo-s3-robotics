// Copyright 2026 meituan_challenge_ws contributors

#include "mtc_core_cpp/error_codes.hpp"

#include <map>
#include <string>

#include "mtc_core_cpp/tool_state.hpp"

namespace mtc
{
namespace core
{

namespace
{

const std::map<uint16_t, const char *> & code_names()
{
  static const std::map<uint16_t, const char *> kNames = {
    {ErrorCodes::OK, "OK"},
    {ErrorCodes::INVALID_TASK, "INVALID_TASK"},
    {ErrorCodes::TARGET_NOT_FOUND, "TARGET_NOT_FOUND"},
    {ErrorCodes::POSE_STALE, "POSE_STALE"},
    {ErrorCodes::PLANNING_FAILED, "PLANNING_FAILED"},
    {ErrorCodes::EXECUTION_REJECTED, "EXECUTION_REJECTED"},
    {ErrorCodes::MOTION_TIMEOUT, "MOTION_TIMEOUT"},
    {ErrorCodes::MOTION_STATUS_UNKNOWN, "MOTION_STATUS_UNKNOWN"},
    {ErrorCodes::ENGAGE_FAILED, "ENGAGE_FAILED"},
    {ErrorCodes::ATTACH_NOT_VERIFIED, "ATTACH_NOT_VERIFIED"},
    {ErrorCodes::OBJECT_DROPPED, "OBJECT_DROPPED"},
    {ErrorCodes::SEAT_NOT_VERIFIED, "SEAT_NOT_VERIFIED"},
    {ErrorCodes::UNLOCK_FAILED, "UNLOCK_FAILED"},
    {ErrorCodes::RELEASE_NOT_VERIFIED, "RELEASE_NOT_VERIFIED"},
    {ErrorCodes::PLACEMENT_FAILED, "PLACEMENT_FAILED"},
    {ErrorCodes::SAFETY_INTERLOCK, "SAFETY_INTERLOCK"},
    {ErrorCodes::HARDWARE_FAULT, "HARDWARE_FAULT"},
    {ErrorCodes::CANCEL_NOT_CONFIRMED, "CANCEL_NOT_CONFIRMED"},
  };
  return kNames;
}

}  // namespace

std::string ErrorCodes::name_of(uint16_t code)
{
  const auto & names = code_names();
  const auto it = names.find(code);
  if (it != names.end()) {
    return std::string(it->second);
  }
  return "UNKNOWN_" + std::to_string(static_cast<unsigned int>(code));
}

bool ErrorCodes::is_known(uint16_t code)
{
  return code_names().find(code) != code_names().end();
}

std::string ToolStateCode::name_of(uint8_t state)
{
  switch (state) {
    case STATE_UNKNOWN: return "STATE_UNKNOWN";
    case STATE_DETACHED: return "STATE_DETACHED";
    case STATE_ENGAGING: return "STATE_ENGAGING";
    case STATE_ATTACHED: return "STATE_ATTACHED";
    case STATE_RELEASING: return "STATE_RELEASING";
    case STATE_FAULT: return "STATE_FAULT";
    default: break;
  }
  return "UNKNOWN_STATE_" + std::to_string(static_cast<unsigned int>(state));
}

std::string EvidenceLevel::name_of(uint8_t level)
{
  switch (level) {
    case NONE: return "EVIDENCE_NONE";
    case COMMAND_ONLY: return "EVIDENCE_COMMAND_ONLY";
    case GEOMETRY: return "EVIDENCE_GEOMETRY";
    case SENSOR_OR_VISION: return "EVIDENCE_SENSOR_OR_VISION";
    default: break;
  }
  return "UNKNOWN_EVIDENCE_" + std::to_string(static_cast<unsigned int>(level));
}

}  // namespace core
}  // namespace mtc

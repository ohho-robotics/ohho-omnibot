// Copyright 2026 OhhO Robotics
// SPDX-License-Identifier: Apache-2.0

#include "omnibot_hardware/feetech_protocol.hpp"

#include <cmath>

namespace omnibot_hardware
{
namespace feetech
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kSpeedStepRpm = 0.0146;

}  // namespace

std::vector<uint8_t> build(uint8_t id, uint8_t inst, const std::vector<uint8_t> & params)
{
  const auto len = static_cast<uint8_t>(params.size() + 2);
  std::vector<uint8_t> packet{0xFF, 0xFF, id, len, inst};
  packet.insert(packet.end(), params.begin(), params.end());
  unsigned sum = static_cast<unsigned>(id) + len + inst;
  for (const auto byte : params) {
    sum += byte;
  }
  packet.push_back(static_cast<uint8_t>(~sum));
  return packet;
}

std::vector<uint8_t> write_torque(uint8_t id, bool enabled)
{
  return build(id, kInstWrite, {kAddrTorqueEnable, static_cast<uint8_t>(enabled ? 1 : 0)});
}

std::vector<uint8_t> write_goal(uint8_t id, int ticks)
{
  if (ticks < 0) {
    ticks = 0;
  }
  if (ticks > 4095) {
    ticks = 4095;
  }
  const auto bits = static_cast<uint16_t>(ticks);
  return build(
    id, kInstWrite,
    {kAddrGoalPosition, static_cast<uint8_t>(bits & 0xFF), static_cast<uint8_t>((bits >> 8) & 0xFF)});
}

std::vector<uint8_t> read_position_speed(uint8_t id)
{
  // Present_Position (2) then Present_Speed (2).
  return build(id, kInstRead, {kAddrPresentPosition, 4});
}

double speed_to_rad_s(uint16_t raw)
{
  const int sign = (raw & 0x8000) ? -1 : 1;
  const int magnitude = raw & 0x7FFF;
  const double rpm = static_cast<double>(magnitude) * kSpeedStepRpm;
  return sign * rpm * (2.0 * kPi / 60.0);
}

std::vector<Status> consume_statuses(std::vector<uint8_t> & buffer)
{
  std::vector<Status> out;
  size_t index = 0;
  while (index + 4 < buffer.size()) {
    if (buffer[index] != 0xFF || buffer[index + 1] != 0xFF) {
      ++index;
      continue;
    }
    const uint8_t id = buffer[index + 2];
    const uint8_t len = buffer[index + 3];
    const size_t total = static_cast<size_t>(4 + len);
    if (index + total > buffer.size()) {
      break;
    }
    unsigned sum = static_cast<unsigned>(id) + len;
    for (size_t k = 0; k + 1 < len; ++k) {
      sum += buffer[index + 4 + k];
    }
    const auto checksum = static_cast<uint8_t>(~sum);
    if (checksum != buffer[index + 4 + len - 1]) {
      ++index;
      continue;
    }
    Status status;
    status.id = id;
    status.error = buffer[index + 4];
    if (len > 2) {
      status.params.assign(
        buffer.begin() + static_cast<std::ptrdiff_t>(index + 5),
        buffer.begin() + static_cast<std::ptrdiff_t>(index + 4 + len - 1));
    }
    out.push_back(status);
    index += total;
  }
  buffer.erase(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(index));
  return out;
}

}  // namespace feetech
}  // namespace omnibot_hardware

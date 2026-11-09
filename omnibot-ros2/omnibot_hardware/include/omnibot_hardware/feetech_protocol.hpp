// Copyright 2026 OhhO Robotics
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNIBOT_HARDWARE__FEETECH_PROTOCOL_HPP_
#define OMNIBOT_HARDWARE__FEETECH_PROTOCOL_HPP_

#include <cstdint>
#include <vector>

namespace omnibot_hardware
{
namespace feetech
{

// STS3215 protocol 0, the same family as LeRobot's FeetechMotorsBus.
// Packet: FF FF ID LEN INST PARAMS... CHECKSUM
// LEN = n_params + 2. Checksum = ~(ID + LEN + INST + params) & 0xFF.
// Status: FF FF ID LEN ERR PARAMS... CS. Total size = 4 + LEN.

constexpr uint8_t kInstRead = 0x02;
constexpr uint8_t kInstWrite = 0x03;
constexpr uint8_t kAddrTorqueEnable = 40;
constexpr uint8_t kAddrGoalPosition = 42;
constexpr uint8_t kAddrPresentPosition = 56;
constexpr uint8_t kAddrPresentSpeed = 58;

struct Status
{
  uint8_t id{0};
  uint8_t error{0};
  std::vector<uint8_t> params;
};

std::vector<uint8_t> build(uint8_t id, uint8_t inst, const std::vector<uint8_t> & params);
std::vector<uint8_t> write_torque(uint8_t id, bool enabled);
std::vector<uint8_t> write_goal(uint8_t id, int ticks);
std::vector<uint8_t> read_position_speed(uint8_t id);

// Present speed: bit 15 is direction, magnitude is steps of 0.0146 rpm.
double speed_to_rad_s(uint16_t raw);

std::vector<Status> consume_statuses(std::vector<uint8_t> & buffer);

}  // namespace feetech
}  // namespace omnibot_hardware

#endif  // OMNIBOT_HARDWARE__FEETECH_PROTOCOL_HPP_

// Copyright 2026 OhhO Robotics
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNIBOT_HARDWARE__YAHBOOM_PROTOCOL_HPP_
#define OMNIBOT_HARDWARE__YAHBOOM_PROTOCOL_HPP_

#include <cstdint>
#include <vector>

namespace omnibot_hardware
{
namespace yahboom
{

// Framing matches yahboom-python-driver/yahboom_ros2/protocol.py.
// The board has no per-wheel encoder opcode. FUNC_MOTION is the framed
// body-velocity command. TYPE_VELOCITY is the framed body-velocity report.

constexpr uint8_t kHeadTx = 0xFF;
constexpr uint8_t kDeviceId = 0xFC;
constexpr uint8_t kHeadRx = 0xFB;
constexpr uint8_t kFuncMotion = 0x12;
constexpr uint8_t kFuncSetCarType = 0x15;
constexpr uint8_t kTypeVelocity = 0x0C;
constexpr uint8_t kCarTypeMecanumX3 = 1;

struct BodyVelocity
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

std::vector<uint8_t> build_packet(uint8_t func, const std::vector<uint8_t> & payload);
std::vector<uint8_t> packet_set_car_type(uint8_t car_type = kCarTypeMecanumX3);
std::vector<uint8_t> packet_motion(double vx, double vy, double wz, uint8_t car_type = kCarTypeMecanumX3);

// Drops complete frames from buffer and returns every TYPE_VELOCITY in order.
// A bad checksum is skipped one byte at a time. An incomplete tail stays.
std::vector<BodyVelocity> consume_velocity(std::vector<uint8_t> & buffer);

}  // namespace yahboom
}  // namespace omnibot_hardware

#endif  // OMNIBOT_HARDWARE__YAHBOOM_PROTOCOL_HPP_

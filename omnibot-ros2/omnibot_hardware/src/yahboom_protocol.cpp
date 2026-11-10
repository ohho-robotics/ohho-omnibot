// Copyright 2026 OhhO Robotics
// SPDX-License-Identifier: Apache-2.0

#include "omnibot_hardware/yahboom_protocol.hpp"

namespace omnibot_hardware
{
namespace yahboom
{
namespace
{

void append_i16(std::vector<uint8_t> & out, int value)
{
  if (value > 32767) {
    value = 32767;
  }
  if (value < -32768) {
    value = -32768;
  }
  const auto bits = static_cast<uint16_t>(static_cast<int16_t>(value));
  out.push_back(static_cast<uint8_t>(bits & 0xFF));
  out.push_back(static_cast<uint8_t>((bits >> 8) & 0xFF));
}

int16_t read_i16(const std::vector<uint8_t> & buffer, size_t offset)
{
  const uint16_t bits = static_cast<uint16_t>(buffer[offset] | (buffer[offset + 1] << 8));
  return static_cast<int16_t>(bits);
}

}  // namespace

std::vector<uint8_t> build_packet(uint8_t func, const std::vector<uint8_t> & payload)
{
  // LEN = ID + LEN + FUNC + payload. Checksum = (sum(bytes) + 5) & 0xFF.
  std::vector<uint8_t> packet;
  packet.reserve(5 + payload.size());
  packet.push_back(kHeadTx);
  packet.push_back(kDeviceId);
  packet.push_back(static_cast<uint8_t>(3 + payload.size()));
  packet.push_back(func);
  packet.insert(packet.end(), payload.begin(), payload.end());
  unsigned sum = 0;
  for (const auto byte : packet) {
    sum += byte;
  }
  packet.push_back(static_cast<uint8_t>((sum + 5) & 0xFF));
  return packet;
}

std::vector<uint8_t> packet_set_car_type(uint8_t car_type)
{
  return build_packet(kFuncSetCarType, {car_type});
}

std::vector<uint8_t> packet_motion(double vx, double vy, double wz, uint8_t car_type)
{
  // int() in the Python driver truncates toward zero.
  std::vector<uint8_t> payload;
  payload.push_back(car_type);
  append_i16(payload, static_cast<int>(vx * 1000.0));
  append_i16(payload, static_cast<int>(vy * 1000.0));
  append_i16(payload, static_cast<int>(wz * 1000.0));
  return build_packet(kFuncMotion, payload);
}

std::vector<BodyVelocity> consume_velocity(std::vector<uint8_t> & buffer)
{
  std::vector<BodyVelocity> out;
  size_t index = 0;
  while (index + 3 < buffer.size()) {
    if (buffer[index] != kHeadTx || buffer[index + 1] != kHeadRx) {
      ++index;
      continue;
    }
    const uint8_t length = buffer[index + 2];
    const size_t total = static_cast<size_t>(2 + length);
    if (index + total > buffer.size()) {
      break;
    }
    unsigned sum = 0;
    for (size_t k = 2; k + 1 < total; ++k) {
      sum += buffer[index + k];
    }
    if (static_cast<uint8_t>(sum & 0xFF) != buffer[index + total - 1]) {
      ++index;
      continue;
    }
    const uint8_t type = buffer[index + 3];
    if (type == kTypeVelocity && total >= 11) {
      BodyVelocity velocity;
      velocity.vx = static_cast<double>(read_i16(buffer, index + 4)) / 1000.0;
      velocity.vy = static_cast<double>(read_i16(buffer, index + 6)) / 1000.0;
      velocity.wz = static_cast<double>(read_i16(buffer, index + 8)) / 1000.0;
      out.push_back(velocity);
    }
    index += total;
  }
  buffer.erase(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(index));
  return out;
}

}  // namespace yahboom
}  // namespace omnibot_hardware

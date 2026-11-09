// Copyright 2026 OhhO Robotics
// SPDX-License-Identifier: Apache-2.0

#include "omnibot_hardware/hardware_buses.hpp"

#include <chrono>
#include <thread>

#include "omnibot_hardware/feetech_protocol.hpp"
#include "omnibot_hardware/kinematics.hpp"
#include "omnibot_hardware/tick_math.hpp"
#include "omnibot_hardware/yahboom_protocol.hpp"

namespace omnibot_hardware
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

}  // namespace

HardwareBuses::HardwareBuses(SerialPort & base, SerialPort & arm, BusConfig config)
: base_(base),
  arm_(arm),
  config_(std::move(config)),
  ticks_per_radian_(tick_math::ticks_per_rad(config_.ticks_per_rev)),
  length_sum_(kinematics::length_sum(config_.wheel_separation_length, config_.wheel_separation_width))
{
}

bool HardwareBuses::configure(std::string & error)
{
  const auto packet = yahboom::packet_set_car_type();
  for (int attempt = 0; attempt < 5; ++attempt) {
    if (!base_.write_bytes(packet, error)) {
      return false;
    }
    if (config_.pace_car_type && attempt + 1 < 5) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }
  return true;
}

bool HardwareBuses::capture_arm_hold(std::string & error)
{
  for (int i = 0; i < 6; ++i) {
    const auto id = static_cast<uint8_t>(config_.servo_ids[static_cast<size_t>(i)]);
    arm_rx_.clear();
    if (!arm_.write_bytes(feetech::read_position_speed(id), error)) {
      return false;
    }
    const auto bytes = arm_.read_for(80);
    arm_rx_.insert(arm_rx_.end(), bytes.begin(), bytes.end());
    const auto statuses = feetech::consume_statuses(arm_rx_);
    const feetech::Status * match = nullptr;
    for (const auto & status : statuses) {
      if (status.id == id && status.error == 0 && status.params.size() >= 2) {
        match = &status;
      }
    }
    if (match == nullptr) {
      error = "no present position from servo " + std::to_string(id);
      return false;
    }
    const int ticks = static_cast<int>(match->params[0] | (match->params[1] << 8));
    hold_ticks_[static_cast<size_t>(i)] = ticks;
    arm_position_[static_cast<size_t>(i)] = tick_math::ticks_to_radians(
      ticks, config_.home_ticks[static_cast<size_t>(i)], ticks_per_radian_);
    if (match->params.size() >= 4) {
      const uint16_t raw = static_cast<uint16_t>(match->params[2] | (match->params[3] << 8));
      arm_velocity_[static_cast<size_t>(i)] = feetech::speed_to_rad_s(raw);
    }
  }
  for (int i = 0; i < 6; ++i) {
    const auto id = static_cast<uint8_t>(config_.servo_ids[static_cast<size_t>(i)]);
    if (!arm_.write_bytes(feetech::write_goal(id, hold_ticks_[static_cast<size_t>(i)]), error)) {
      return false;
    }
  }
  return set_arm_torque(true, error);
}

bool HardwareBuses::write_wheel_velocities(
  const std::array<double, 4> & rad_s, std::string & error)
{
  const auto body = kinematics::wheels_to_body(rad_s, config_.wheel_radius, length_sum_);
  return base_.write_bytes(yahboom::packet_motion(body.vx, body.vy, body.wz), error);
}

bool HardwareBuses::write_arm_positions(
  const std::array<double, 6> & radians, std::string & error)
{
  for (int i = 0; i < 6; ++i) {
    const int ticks = tick_math::radians_to_ticks(
      radians[static_cast<size_t>(i)], config_.home_ticks[static_cast<size_t>(i)], ticks_per_radian_);
    const auto id = static_cast<uint8_t>(config_.servo_ids[static_cast<size_t>(i)]);
    if (!arm_.write_bytes(feetech::write_goal(id, ticks), error)) {
      return false;
    }
  }
  return true;
}

bool HardwareBuses::set_arm_torque(bool enabled, std::string & error)
{
  for (int i = 0; i < 6; ++i) {
    const auto id = static_cast<uint8_t>(config_.servo_ids[static_cast<size_t>(i)]);
    if (!arm_.write_bytes(feetech::write_torque(id, enabled), error)) {
      return false;
    }
  }
  torque_on_ = enabled;
  return true;
}

bool HardwareBuses::request_arm_feedback(std::string & error)
{
  for (int i = 0; i < 6; ++i) {
    const auto id = static_cast<uint8_t>(config_.servo_ids[static_cast<size_t>(i)]);
    if (!arm_.write_bytes(feetech::read_position_speed(id), error)) {
      return false;
    }
  }
  return true;
}

void HardwareBuses::poll_base(double dt_sec)
{
  const auto bytes = base_.read_available();
  base_rx_.insert(base_rx_.end(), bytes.begin(), bytes.end());
  const auto samples = yahboom::consume_velocity(base_rx_);
  if (samples.empty()) {
    wheel_velocity_.fill(0.0);
    return;
  }
  if (dt_sec < 0.0) {
    dt_sec = 0.0;
  }
  if (dt_sec > 0.1) {
    dt_sec = 0.1;
  }
  const auto wheels = kinematics::body_to_wheels(samples.back(), config_.wheel_radius, length_sum_);
  const double ticks_per_radian = config_.encoder_ticks_per_revolution / (2.0 * kPi);
  for (int i = 0; i < 4; ++i) {
    wheel_velocity_[static_cast<size_t>(i)] = wheels[static_cast<size_t>(i)];
    encoder_ticks_[static_cast<size_t>(i)] += wheels[static_cast<size_t>(i)] * dt_sec * ticks_per_radian;
    wheel_position_[static_cast<size_t>(i)] =
      encoder_ticks_[static_cast<size_t>(i)] * (2.0 * kPi) / config_.encoder_ticks_per_revolution;
  }
}

void HardwareBuses::poll_arm()
{
  const auto bytes = arm_.read_available();
  arm_rx_.insert(arm_rx_.end(), bytes.begin(), bytes.end());
  const auto statuses = feetech::consume_statuses(arm_rx_);
  for (const auto & status : statuses) {
    if (status.error != 0 || status.params.size() < 2) {
      continue;
    }
    for (int i = 0; i < 6; ++i) {
      if (config_.servo_ids[static_cast<size_t>(i)] != static_cast<int>(status.id)) {
        continue;
      }
      const int ticks = static_cast<int>(status.params[0] | (status.params[1] << 8));
      arm_position_[static_cast<size_t>(i)] = tick_math::ticks_to_radians(
        ticks, config_.home_ticks[static_cast<size_t>(i)], ticks_per_radian_);
      if (status.params.size() >= 4) {
        const uint16_t raw = static_cast<uint16_t>(status.params[2] | (status.params[3] << 8));
        arm_velocity_[static_cast<size_t>(i)] = feetech::speed_to_rad_s(raw);
      }
    }
  }
}

bool HardwareBuses::stop(std::string & error)
{
  wheel_velocity_.fill(0.0);
  const bool base_ok = base_.write_bytes(yahboom::packet_motion(0.0, 0.0, 0.0), error);
  std::string torque_error;
  const bool torque_ok = set_arm_torque(false, torque_error);
  if (!base_ok) {
    return false;
  }
  if (!torque_ok) {
    error = torque_error;
    return false;
  }
  return true;
}

}  // namespace omnibot_hardware

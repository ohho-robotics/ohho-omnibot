// Copyright 2026 OhhO Robotics
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNIBOT_HARDWARE__HARDWARE_BUSES_HPP_
#define OMNIBOT_HARDWARE__HARDWARE_BUSES_HPP_

#include <array>
#include <string>
#include <utility>
#include <vector>

#include "omnibot_hardware/serial_port.hpp"

namespace omnibot_hardware
{

struct BusConfig
{
  double wheel_radius{0.04};
  double wheel_separation_length{0.165};
  double wheel_separation_width{0.215};
  double encoder_ticks_per_revolution{1440.0};
  std::array<int, 6> servo_ids{1, 2, 3, 4, 5, 6};
  std::array<int, 6> home_ticks{2048, 2048, 2048, 2048, 2048, 2048};
  double ticks_per_rev{4096.0};
  // The Python driver sleeps 50 ms between the five SET_CAR_TYPE frames.
  // Tests set this false so they do not wait.
  bool pace_car_type{true};
};

// Serial sessions for the base and the arm. Does not open ports.
// Wheel order is front-left, front-right, rear-left, rear-right.
class HardwareBuses
{
public:
  HardwareBuses(SerialPort & base, SerialPort & arm, BusConfig config);

  bool configure(std::string & error);
  // Read present position from each servo. Fail if any read fails.
  // Then write that position as the goal and enable torque, so the first
  // command is a hold. Does not command tick 0 when a read fails.
  bool capture_arm_hold(std::string & error);
  bool write_wheel_velocities(const std::array<double, 4> & rad_s, std::string & error);
  bool write_arm_positions(const std::array<double, 6> & radians, std::string & error);
  bool set_arm_torque(bool enabled, std::string & error);
  bool request_arm_feedback(std::string & error);
  void poll_base(double dt_sec);
  void poll_arm();
  bool stop(std::string & error);

  const std::array<double, 4> & wheel_position() const { return wheel_position_; }
  const std::array<double, 4> & wheel_velocity() const { return wheel_velocity_; }
  const std::array<double, 6> & arm_position() const { return arm_position_; }
  const std::array<double, 6> & arm_velocity() const { return arm_velocity_; }
  const std::array<int, 6> & hold_ticks() const { return hold_ticks_; }
  bool torque_on() const { return torque_on_; }

private:
  SerialPort & base_;
  SerialPort & arm_;
  BusConfig config_;
  double ticks_per_radian_;
  double length_sum_;
  std::vector<uint8_t> base_rx_;
  std::vector<uint8_t> arm_rx_;
  std::array<double, 4> wheel_position_{};
  std::array<double, 4> wheel_velocity_{};
  std::array<double, 4> encoder_ticks_{};
  std::array<double, 6> arm_position_{};
  std::array<double, 6> arm_velocity_{};
  std::array<int, 6> hold_ticks_{};
  bool torque_on_{false};
};

}  // namespace omnibot_hardware

#endif  // OMNIBOT_HARDWARE__HARDWARE_BUSES_HPP_

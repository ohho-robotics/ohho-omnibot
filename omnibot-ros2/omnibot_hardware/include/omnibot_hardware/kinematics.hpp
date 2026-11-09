// Copyright 2026 OhhO Robotics
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNIBOT_HARDWARE__KINEMATICS_HPP_
#define OMNIBOT_HARDWARE__KINEMATICS_HPP_

#include <array>

#include "omnibot_hardware/yahboom_protocol.hpp"

namespace omnibot_hardware
{
namespace kinematics
{

// Wheel order is front-left, front-right, rear-left, rear-right.
// length_sum is lx + ly, the mecanum controller's
// sum_of_robot_center_projection_on_X_Y_axis.
//   w_fl = (vx - vy - L * wz) / r
//   w_fr = (vx + vy + L * wz) / r
//   w_rl = (vx + vy - L * wz) / r
//   w_rr = (vx - vy + L * wz) / r

std::array<double, 4> body_to_wheels(
  const yahboom::BodyVelocity & body, double wheel_radius, double length_sum);

yahboom::BodyVelocity wheels_to_body(
  const std::array<double, 4> & wheel_rad_s, double wheel_radius, double length_sum);

// Full separations from the URDF (front-back, left-right). lx and ly are halves.
double length_sum(double wheel_separation_length, double wheel_separation_width);

}  // namespace kinematics
}  // namespace omnibot_hardware

#endif  // OMNIBOT_HARDWARE__KINEMATICS_HPP_

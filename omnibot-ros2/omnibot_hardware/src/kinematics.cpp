// Copyright 2026 OhhO Robotics
// SPDX-License-Identifier: Apache-2.0

#include "omnibot_hardware/kinematics.hpp"

namespace omnibot_hardware
{
namespace kinematics
{

double length_sum(double wheel_separation_length, double wheel_separation_width)
{
  return (wheel_separation_length + wheel_separation_width) / 2.0;
}

std::array<double, 4> body_to_wheels(
  const yahboom::BodyVelocity & body, double wheel_radius, double length_sum_m)
{
  const double r = wheel_radius;
  const double l = length_sum_m;
  return {
    (body.vx - body.vy - l * body.wz) / r,
    (body.vx + body.vy + l * body.wz) / r,
    (body.vx + body.vy - l * body.wz) / r,
    (body.vx - body.vy + l * body.wz) / r,
  };
}

yahboom::BodyVelocity wheels_to_body(
  const std::array<double, 4> & wheel_rad_s, double wheel_radius, double length_sum_m)
{
  const double fl = wheel_rad_s[0];
  const double fr = wheel_rad_s[1];
  const double rl = wheel_rad_s[2];
  const double rr = wheel_rad_s[3];
  yahboom::BodyVelocity body;
  body.vx = wheel_radius / 4.0 * (fl + fr + rl + rr);
  body.vy = wheel_radius / 4.0 * (-fl + fr + rl - rr);
  body.wz = wheel_radius / (4.0 * length_sum_m) * (-fl + fr - rl + rr);
  return body;
}

}  // namespace kinematics
}  // namespace omnibot_hardware

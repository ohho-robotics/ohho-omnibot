// Copyright 2026 OhhO Robotics
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNIBOT_HARDWARE__TICK_MATH_HPP_
#define OMNIBOT_HARDWARE__TICK_MATH_HPP_

namespace omnibot_hardware
{
namespace tick_math
{

// Same conversion as omnibot_arm/scripts/arm_math.py:
//   ticks_per_rad = ticks_per_rev / (2 * pi)
//   radians = (ticks - home_ticks) / ticks_per_rad
//   ticks = round(radians * ticks_per_rad + home_ticks), then clamp 0..4095.

constexpr int kTickMin = 0;
constexpr int kTickMax = 4095;

double ticks_per_rad(double ticks_per_rev);
double ticks_to_radians(int ticks, int home_ticks, double ticks_per_radian);
int radians_to_ticks(double radians, int home_ticks, double ticks_per_radian);

}  // namespace tick_math
}  // namespace omnibot_hardware

#endif  // OMNIBOT_HARDWARE__TICK_MATH_HPP_

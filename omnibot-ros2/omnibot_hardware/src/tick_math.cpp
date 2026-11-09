// Copyright 2026 OhhO Robotics
// SPDX-License-Identifier: Apache-2.0

#include "omnibot_hardware/tick_math.hpp"

#include <cmath>

namespace omnibot_hardware
{
namespace tick_math
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

int clamp_tick(int ticks)
{
  if (ticks < kTickMin) {
    return kTickMin;
  }
  if (ticks > kTickMax) {
    return kTickMax;
  }
  return ticks;
}

}  // namespace

double ticks_per_rad(double ticks_per_rev)
{
  return ticks_per_rev / (2.0 * kPi);
}

double ticks_to_radians(int ticks, int home_ticks, double ticks_per_radian)
{
  return (static_cast<double>(ticks) - static_cast<double>(home_ticks)) / ticks_per_radian;
}

int radians_to_ticks(double radians, int home_ticks, double ticks_per_radian)
{
  const auto ticks = static_cast<int>(std::lround(radians * ticks_per_radian + home_ticks));
  return clamp_tick(ticks);
}

}  // namespace tick_math
}  // namespace omnibot_hardware

// Copyright 2026 OhhO Robotics
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNIBOT_HARDWARE__CALIBRATION_HPP_
#define OMNIBOT_HARDWARE__CALIBRATION_HPP_

#include <array>
#include <string>

namespace omnibot_hardware
{
namespace calibration
{

// Small reader for the OHH-94 yaml. No yaml-cpp.
// home_ticks is exactly six integers in 0..4095, joint order
// arm_shoulder_pan, arm_shoulder_lift, arm_elbow_flex, arm_wrist_flex,
// arm_wrist_roll, arm_gripper.
// When a top-level joints list is present, each home_tick must equal the
// matching home_ticks entry. The key home_ticks is not a home_tick.
bool parse_home_ticks_yaml(
  const std::string & text, std::array<int, 6> & home_ticks, std::string & error);

bool load_home_ticks_file(
  const std::string & path, std::array<int, 6> & home_ticks, std::string & error);

}  // namespace calibration
}  // namespace omnibot_hardware

#endif  // OMNIBOT_HARDWARE__CALIBRATION_HPP_

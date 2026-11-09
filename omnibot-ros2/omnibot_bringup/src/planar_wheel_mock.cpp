// Copyright 2026 OhhO Robotics
// SPDX-License-Identifier: Apache-2.0

#include <cmath>
#include <map>
#include <memory>
#include <string>

#include "gz_ros2_control/gz_system_interface.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace omnibot_bringup
{

// gz_ros2_control's resource manager only loads GazeboSimSystemInterface.
// mock_components/GenericSystem is a hardware_interface::SystemInterface, so
// the planar wheel block never came up and the mecanum controller stayed
// inactive. This plugin exports the same velocity command and position and
// velocity state, and it does not write Gazebo joint commands. VelocityControl
// keeps the kinematic base.
class PlanarWheelMock : public gz_ros2_control::GazeboSimSystemInterface
{
public:
  bool initSim(
    rclcpp::Node::SharedPtr & /*model_nh*/,
    std::map<std::string, sim::Entity> & /*joints*/,
    const hardware_interface::HardwareInfo & /*hardware_info*/,
    sim::EntityComponentManager & /*ecm*/,
    unsigned int /*update_rate*/) override
  {
    return true;
  }

  hardware_interface::return_type read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & period) override
  {
    const double dt = period.seconds();
    for (const auto & joint : info_.joints) {
      const std::string velocity = joint.name + "/velocity";
      const std::string position = joint.name + "/position";
      if (!has_command(velocity)) {
        continue;
      }
      double command = get_command<double>(velocity);
      if (!std::isfinite(command)) {
        command = 0.0;
      }
      if (has_state(velocity)) {
        set_state(velocity, command);
      }
      if (has_state(position)) {
        double state = get_state<double>(position);
        if (!std::isfinite(state)) {
          state = 0.0;
        }
        set_state(position, state + command * dt);
      }
    }
    return hardware_interface::return_type::OK;
  }
};

}  // namespace omnibot_bringup

PLUGINLIB_EXPORT_CLASS(
  omnibot_bringup::PlanarWheelMock, gz_ros2_control::GazeboSimSystemInterface)

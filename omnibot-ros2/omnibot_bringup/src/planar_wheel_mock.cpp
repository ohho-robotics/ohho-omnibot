// Copyright 2026 OhhO Robotics
// SPDX-License-Identifier: Apache-2.0

#include <cmath>
#include <map>
#include <memory>
#include <string>

#include "gz/sim/Entity.hh"
#include "gz/sim/components/JointForceCmd.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz_ros2_control/gz_system_interface.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace omnibot_bringup
{

// gz_ros2_control's resource manager only loads GazeboSimSystemInterface.
// mock_components/GenericSystem is a hardware_interface::SystemInterface, so
// the planar wheel block never came up and the mecanum controller stayed
// inactive. This plugin exports the same velocity command and position and
// velocity state. A non-zero command is not written into Gazebo, so
// VelocityControl keeps the kinematic base. A zero command brakes the wheel
// joint; otherwise the cylinder collision free-rolls and the chassis slides
// while /cmd_vel is zero.
//
// gz-physics Harmonic implements JointVelocityCmd as a DART SERVO. The servo
// stays at the last command after the component is removed, so a zero brake
// latches and teleop cannot move the base. JointForceCmd(0) switches that
// actuator back to FORCE. Physics applies force ahead of velocity, so the
// brake also drops the force component or the velocity command never lands.
class PlanarWheelMock : public gz_ros2_control::GazeboSimSystemInterface
{
public:
  bool initSim(
    rclcpp::Node::SharedPtr & /*model_nh*/,
    std::map<std::string, sim::Entity> & joints,
    const hardware_interface::HardwareInfo & hardware_info,
    sim::EntityComponentManager & ecm,
    unsigned int /*update_rate*/) override
  {
    ecm_ = &ecm;
    for (const auto & joint : hardware_info.joints) {
      const auto found = joints.find(joint.name);
      if (found != joints.end()) {
        sim_joints_[joint.name] = found->second;
      }
    }
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
      // Unclaimed wheels stay free so VelocityControl can move the base
      // before the mecanum controller is active. Braking then would lock it.
      if (!has_command(velocity)) {
        release_wheel(joint.name);
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
      // A non-zero wheel speed would fight VelocityControl. Do not write it.
      // Release the brake instead, or the latched servo keeps the wheel at 0.
      if (std::abs(command) >= 1e-3) {
        release_wheel(joint.name);
        continue;
      }
      brake_wheel(joint.name);
    }
    return hardware_interface::return_type::OK;
  }

private:
  sim::Entity joint_entity(const std::string & name) const
  {
    const auto found = sim_joints_.find(name);
    if (found == sim_joints_.end() || ecm_ == nullptr) {
      return sim::kNullEntity;
    }
    return found->second;
  }

  void brake_wheel(const std::string & name)
  {
    const sim::Entity entity = joint_entity(name);
    if (entity == sim::kNullEntity) {
      return;
    }
    // Force wins over velocity in the physics step. The release path drops
    // the force component after the actuator has switched, so this brake is
    // the only command physics sees.
    if (ecm_->Component<sim::components::JointForceCmd>(entity) != nullptr) {
      ecm_->RemoveComponent<sim::components::JointForceCmd>(entity);
    }
    auto * vel = ecm_->Component<sim::components::JointVelocityCmd>(entity);
    if (vel == nullptr) {
      ecm_->CreateComponent(entity, sim::components::JointVelocityCmd({0.0}));
    } else if (!vel->Data().empty()) {
      vel->Data()[0] = 0.0;
    } else {
      vel->Data().push_back(0.0);
    }
    released_[name] = false;
  }

  void release_wheel(const std::string & name)
  {
    const sim::Entity entity = joint_entity(name);
    if (entity == sim::kNullEntity) {
      return;
    }
    if (ecm_->Component<sim::components::JointVelocityCmd>(entity) != nullptr) {
      ecm_->RemoveComponent<sim::components::JointVelocityCmd>(entity);
    }
    // One zero-force command switches the DART actuator off the latched
    // velocity servo. Leaving that component in place would hide the next
    // brake for a step, so drop it once the switch has been requested.
    if (!released_[name]) {
      auto * force = ecm_->Component<sim::components::JointForceCmd>(entity);
      if (force == nullptr) {
        ecm_->CreateComponent(entity, sim::components::JointForceCmd({0.0}));
      } else if (!force->Data().empty()) {
        force->Data()[0] = 0.0;
      } else {
        force->Data().push_back(0.0);
      }
      released_[name] = true;
      return;
    }
    if (ecm_->Component<sim::components::JointForceCmd>(entity) != nullptr) {
      ecm_->RemoveComponent<sim::components::JointForceCmd>(entity);
    }
  }

  sim::EntityComponentManager * ecm_{nullptr};
  std::map<std::string, sim::Entity> sim_joints_;
  std::map<std::string, bool> released_;
};

}  // namespace omnibot_bringup

PLUGINLIB_EXPORT_CLASS(
  omnibot_bringup::PlanarWheelMock, gz_ros2_control::GazeboSimSystemInterface)

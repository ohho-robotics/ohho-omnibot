// Copyright 2026 OhhO Robotics
// SPDX-License-Identifier: Apache-2.0

#include <cmath>
#include <map>
#include <memory>
#include <string>

#include "gz/math/Vector3.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/AngularVelocityCmd.hh"
#include "gz/sim/components/JointForceCmd.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/LinearVelocityCmd.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz_ros2_control/gz_system_interface.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace omnibot_bringup
{

// gz_ros2_control's resource manager only loads GazeboSimSystemInterface.
// mock_components/GenericSystem is a hardware_interface::SystemInterface, so
// the planar wheel block never came up and the mecanum controller stayed
// inactive. This plugin exports the same velocity command and position and
// velocity state. A non-zero command is not written into Gazebo, so
// VelocityControl keeps the kinematic base.
//
// A claimed zero is a hold: brake the rollers and clear the latched body
// twist. VelocityControl writes that twist every PreUpdate, and
// OdometryPublisher averages the last 10 pose steps, so /odom stays above
// the stop threshold until the body command is actually zero. The control
// plugin's gz:system_priority is later than VelocityControl, so the clear
// wins. Do not skip the clear because a gz twist component is still set.
// That component is the latch, and leaving it drives the chassis after
// /cmd_vel goes silent.
//
// A live command releases every wheel. gz-physics Harmonic implements
// JointVelocityCmd as a DART SERVO, and a servo left at 0 locks the roller.
// Locked rollers scrub the chassis under VelocityControl, so the base lags
// the commanded twist: Nav2 can report success while ground truth is still
// outside the 0.30 m check, and a stall trips the progress checker.
// JointForceCmd(0) stops VelocityControl if that force is left on the
// joint, so release uses it once to leave SERVO mode and the next write
// removes it. An unclaimed wheel is released so the base can move before
// the controller claims it.
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
    bool moving = false;
    sim::Entity sample_joint = sim::kNullEntity;
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
      if (sample_joint == sim::kNullEntity) {
        sample_joint = joint_entity(joint.name);
      }
      // Wheel speed is not written into Gazebo. A live command only marks
      // the body as moving so the brake is released below.
      if (std::abs(command) >= 1e-3) {
        moving = true;
      }
    }
    // A claimed zero is a hold. VelocityControl latches the last gz twist
    // and writes it every PreUpdate. Leaving that latch, or a coast left
    // when the rollers were released, drives the chassis while /cmd_vel is
    // silent. The hold clears the command and the current link velocity.
    // A live command releases every wheel. Braking a moving roller locks
    // it, and the locked roller scrubs the planar chassis. The
    // canonical-link zero from a previous hold is removed on the drive
    // path so it cannot pin the next command. Wheels that are not claimed
    // yet stay released so the base can move before the controller is up.
    if (moving) {
      for (const auto & joint : info_.joints) {
        if (has_command(joint.name + "/velocity")) {
          release_wheel(joint.name);
        }
      }
      drop_link_velocity_cmd(sample_joint);
    } else if (sample_joint != sim::kNullEntity) {
      for (const auto & joint : info_.joints) {
        if (has_command(joint.name + "/velocity")) {
          brake_wheel(joint.name);
        }
      }
      zero_base(sample_joint);
    } else {
      drop_link_velocity_cmd(sample_joint);
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

  sim::Entity model_entity(sim::Entity joint)
  {
    if (model_ != sim::kNullEntity || ecm_ == nullptr) {
      return model_;
    }
    sim::Entity entity = joint;
    for (int hop = 0; hop < 4 && entity != sim::kNullEntity; ++hop) {
      if (ecm_->Component<sim::components::Model>(entity) != nullptr) {
        model_ = entity;
        return model_;
      }
      const auto * parent = ecm_->Component<sim::components::ParentEntity>(entity);
      if (parent == nullptr) {
        break;
      }
      entity = parent->Data();
    }
    return sim::kNullEntity;
  }

  template<typename Cmd>
  void set_zero_cmd(sim::Entity entity)
  {
    auto * cmd = ecm_->Component<Cmd>(entity);
    if (cmd == nullptr) {
      ecm_->CreateComponent(entity, Cmd(gz::math::Vector3d::Zero));
      return;
    }
    cmd->Data() = gz::math::Vector3d::Zero;
  }

  template<typename Velocity>
  void zero_velocity_state(sim::Entity entity)
  {
    auto * velocity = ecm_->Component<Velocity>(entity);
    if (velocity == nullptr) {
      return;
    }
    velocity->Data() = gz::math::Vector3d::Zero;
  }

  void drop_link_velocity_cmd(sim::Entity joint)
  {
    if (ecm_ == nullptr || joint == sim::kNullEntity) {
      return;
    }
    const sim::Entity model = model_entity(joint);
    if (model == sim::kNullEntity) {
      return;
    }
    const sim::Entity link = sim::Model(model).CanonicalLink(*ecm_);
    if (link == sim::kNullEntity || link == model) {
      return;
    }
    if (ecm_->Component<sim::components::LinearVelocityCmd>(link) != nullptr) {
      ecm_->RemoveComponent<sim::components::LinearVelocityCmd>(link);
    }
    if (ecm_->Component<sim::components::AngularVelocityCmd>(link) != nullptr) {
      ecm_->RemoveComponent<sim::components::AngularVelocityCmd>(link);
    }
  }

  void zero_base(sim::Entity joint)
  {
    if (ecm_ == nullptr || joint == sim::kNullEntity) {
      return;
    }
    const sim::Entity model = model_entity(joint);
    if (model == sim::kNullEntity) {
      return;
    }
    // VelocityControl with no link name writes the model. The canonical
    // link is zeroed too so a coast stops in the same step, then removed
    // on the next drive so it cannot pin the body at zero. The current
    // velocity is cleared as well: a roller release can leave the chassis
    // sliding after the command component is already zero.
    set_zero_cmd<sim::components::LinearVelocityCmd>(model);
    set_zero_cmd<sim::components::AngularVelocityCmd>(model);
    zero_velocity_state<sim::components::LinearVelocity>(model);
    zero_velocity_state<sim::components::AngularVelocity>(model);
    const sim::Entity link = sim::Model(model).CanonicalLink(*ecm_);
    if (link != sim::kNullEntity && link != model) {
      set_zero_cmd<sim::components::LinearVelocityCmd>(link);
      set_zero_cmd<sim::components::AngularVelocityCmd>(link);
      zero_velocity_state<sim::components::LinearVelocity>(link);
      zero_velocity_state<sim::components::AngularVelocity>(link);
    }
  }

  sim::EntityComponentManager * ecm_{nullptr};
  sim::Entity model_{sim::kNullEntity};
  std::map<std::string, sim::Entity> sim_joints_;
  std::map<std::string, bool> released_;
};

}  // namespace omnibot_bringup

PLUGINLIB_EXPORT_CLASS(
  omnibot_bringup::PlanarWheelMock, gz_ros2_control::GazeboSimSystemInterface)

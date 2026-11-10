// Copyright 2026 OhhO Robotics
// SPDX-License-Identifier: Apache-2.0

#include <cmath>
#include <map>
#include <memory>
#include <string>

#include "gz/math/Vector3.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/components/AngularVelocityCmd.hh"
#include "gz/sim/components/JointForceCmd.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
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
// VelocityControl keeps the kinematic base. A claimed joint stays on a zero
// velocity brake, including while the controller command is live. A zero
// joint force cancels that body twist, so the live command must not release
// the wheel. An unclaimed wheel is released once so the base can still move
// before the controller claims it.
//
// gz-physics Harmonic implements JointVelocityCmd as a DART SERVO. The servo
// stays at the last command after the component is removed. JointForceCmd(0)
// switches that actuator back to FORCE, which is the unclaimed-wheel path.
// Physics applies force ahead of velocity, so the brake drops the force
// component or the velocity command never lands.
//
// VelocityControl latches the last twist and writes it every PreUpdate.
// OdometryPublisher then averages the last 10 pose steps, so /odom stays
// above the stop threshold until the body velocity is actually zero. A
// claimed stop clears that body command in this same write. The control
// plugin's gz:system_priority is later than VelocityControl, so the clear
// wins. A live wheel command does not clear it, or planar teleop cannot start.
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
    bool stopped = true;
    sim::Entity sample_joint = sim::kNullEntity;
    for (const auto & joint : info_.joints) {
      const std::string velocity = joint.name + "/velocity";
      const std::string position = joint.name + "/position";
      // Unclaimed wheels stay free so VelocityControl can move the base
      // before the mecanum controller is active. Braking then would lock it.
      if (!has_command(velocity)) {
        release_wheel(joint.name);
        stopped = false;
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
      // A non-zero wheel speed would fight VelocityControl. Do not write it.
      // JointForceCmd(0) stops VelocityControl, so keep the zero brake.
      // A live command only marks the wheel moving so the body twist is
      // left alone.
      if (std::abs(command) >= 1e-3) {
        stopped = false;
      }
      brake_wheel(joint.name);
    }
    // A claimed zero is a hold. VelocityControl latches the last gz twist and
    // writes it every PreUpdate, and the mecanum controller's idle command is
    // also zero. Leaving that latch in place drives the chassis while
    // /cmd_vel/out is silent (the nav source must not move the base in teleop).
    // A live wheel command does not clear it: the adapter mirrors a real drive
    // twist onto the wheel reference, and clearing the body would stop planar
    // teleop. The canonical-link zero from a previous hold is removed on that
    // path so it cannot pin the next command.
    if (stopped) {
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
    // on the next drive so it cannot pin the body at zero.
    set_zero_cmd<sim::components::LinearVelocityCmd>(model);
    set_zero_cmd<sim::components::AngularVelocityCmd>(model);
    const sim::Entity link = sim::Model(model).CanonicalLink(*ecm_);
    if (link != sim::kNullEntity && link != model) {
      set_zero_cmd<sim::components::LinearVelocityCmd>(link);
      set_zero_cmd<sim::components::AngularVelocityCmd>(link);
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

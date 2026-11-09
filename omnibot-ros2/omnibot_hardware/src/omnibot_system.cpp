// Copyright 2026 OhhO Robotics
// SPDX-License-Identifier: Apache-2.0

#include "omnibot_hardware/omnibot_system.hpp"

#include <sstream>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "omnibot_hardware/calibration.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace omnibot_hardware
{
namespace
{

const std::array<const char *, 4> kWheelNames = {
  "front_left_wheel_joint",
  "front_right_wheel_joint",
  "rear_left_wheel_joint",
  "rear_right_wheel_joint",
};

const std::array<const char *, 6> kArmNames = {
  "arm_shoulder_pan",
  "arm_shoulder_lift",
  "arm_elbow_flex",
  "arm_wrist_flex",
  "arm_wrist_roll",
  "arm_gripper",
};

const hardware_interface::ComponentInfo * find_joint(
  const hardware_interface::HardwareInfo & info, const std::string & name)
{
  for (const auto & joint : info.joints) {
    if (joint.name == name) {
      return &joint;
    }
  }
  return nullptr;
}

bool has_interface(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces, const std::string & name)
{
  for (const auto & interface : interfaces) {
    if (interface.name == name) {
      return true;
    }
  }
  return false;
}

std::string param_or(
  const hardware_interface::HardwareInfo & info, const std::string & key, const std::string & fallback)
{
  const auto found = info.hardware_parameters.find(key);
  if (found == info.hardware_parameters.end() || found->second.empty()) {
    return fallback;
  }
  return found->second;
}

bool parse_double(const std::string & text, double & value)
{
  try {
    size_t used = 0;
    value = std::stod(text, &used);
    return used == text.size();
  } catch (const std::exception &) {
    return false;
  }
}

bool parse_int_text(const std::string & text, int & value)
{
  try {
    size_t used = 0;
    value = std::stoi(text, &used);
    return used == text.size();
  } catch (const std::exception &) {
    return false;
  }
}

std::vector<int> split_ints(const std::string & text)
{
  std::vector<int> values;
  std::stringstream stream(text);
  std::string item;
  while (std::getline(stream, item, ',')) {
    const auto begin = item.find_first_not_of(" \t");
    if (begin == std::string::npos) {
      continue;
    }
    const auto end = item.find_last_not_of(" \t");
    int value = 0;
    if (!parse_int_text(item.substr(begin, end - begin + 1), value)) {
      return {};
    }
    values.push_back(value);
  }
  return values;
}

}  // namespace

hardware_interface::CallbackReturn OmniBotSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const char * name : kWheelNames) {
    const auto * joint = find_joint(info_, name);
    if (joint == nullptr) {
      RCLCPP_ERROR(get_logger(), "URDF is missing wheel joint %s", name);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (!has_interface(joint->command_interfaces, hardware_interface::HW_IF_VELOCITY) ||
      !has_interface(joint->state_interfaces, hardware_interface::HW_IF_POSITION) ||
      !has_interface(joint->state_interfaces, hardware_interface::HW_IF_VELOCITY))
    {
      RCLCPP_ERROR(get_logger(), "Wheel joint %s needs velocity command and position+velocity state", name);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  for (const char * name : kArmNames) {
    const auto * joint = find_joint(info_, name);
    if (joint == nullptr) {
      RCLCPP_ERROR(get_logger(), "URDF is missing arm joint %s", name);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (!has_interface(joint->command_interfaces, hardware_interface::HW_IF_POSITION) ||
      !has_interface(joint->state_interfaces, hardware_interface::HW_IF_POSITION) ||
      !has_interface(joint->state_interfaces, hardware_interface::HW_IF_VELOCITY))
    {
      RCLCPP_ERROR(get_logger(), "Arm joint %s needs position command and position+velocity state", name);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  base_path_ = param_or(info_, "base_port", "/dev/ttyUSB0");
  arm_path_ = param_or(info_, "arm_port", "/dev/ttyACM0");
  if (!parse_int_text(param_or(info_, "base_baud", "115200"), base_baud_) ||
    !parse_int_text(param_or(info_, "arm_baud", "1000000"), arm_baud_))
  {
    RCLCPP_ERROR(get_logger(), "base_baud and arm_baud must be integers");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!parse_double(param_or(info_, "wheel_radius", "0.04"), config_.wheel_radius) ||
    !parse_double(param_or(info_, "wheel_separation_length", "0.165"), config_.wheel_separation_length) ||
    !parse_double(param_or(info_, "wheel_separation_width", "0.215"), config_.wheel_separation_width) ||
    !parse_double(
      param_or(info_, "encoder_ticks_per_revolution", "1440"), config_.encoder_ticks_per_revolution) ||
    !parse_double(param_or(info_, "ticks_per_rev", "4096"), config_.ticks_per_rev))
  {
    RCLCPP_ERROR(get_logger(), "wheel and tick parameters must be numbers");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (config_.wheel_radius <= 0.0 || config_.encoder_ticks_per_revolution <= 0.0 || config_.ticks_per_rev <= 0.0) {
    RCLCPP_ERROR(get_logger(), "wheel_radius, encoder_ticks_per_revolution, and ticks_per_rev must be > 0");
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto servo_ids = split_ints(param_or(info_, "servo_ids", "1,2,3,4,5,6"));
  const auto home_ticks = split_ints(param_or(info_, "home_ticks", "2048,2048,2048,2048,2048,2048"));
  if (servo_ids.size() != 6 || home_ticks.size() != 6) {
    RCLCPP_ERROR(get_logger(), "servo_ids and home_ticks need six comma-separated integers");
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (int i = 0; i < 6; ++i) {
    config_.servo_ids[static_cast<size_t>(i)] = servo_ids[static_cast<size_t>(i)];
    if (home_ticks[static_cast<size_t>(i)] < 0 || home_ticks[static_cast<size_t>(i)] > 4095) {
      RCLCPP_ERROR(get_logger(), "home_ticks must be in 0..4095");
      return hardware_interface::CallbackReturn::ERROR;
    }
    config_.home_ticks[static_cast<size_t>(i)] = home_ticks[static_cast<size_t>(i)];
  }
  const auto calibration_file = param_or(info_, "calibration_file", "");
  if (!calibration_file.empty()) {
    std::string error;
    if (!calibration::load_home_ticks_file(calibration_file, config_.home_ticks, error)) {
      RCLCPP_ERROR(get_logger(), "%s", error.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(get_logger(), "Loaded SO-101 home ticks from %s", calibration_file.c_str());
  }

  for (int i = 0; i < 4; ++i) {
    wheel_names_[static_cast<size_t>(i)] = kWheelNames[static_cast<size_t>(i)];
  }
  for (int i = 0; i < 6; ++i) {
    arm_names_[static_cast<size_t>(i)] = kArmNames[static_cast<size_t>(i)];
  }
  config_.pace_car_type = true;
  buses_ = std::make_unique<HardwareBuses>(base_port_, arm_port_, config_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

void OmniBotSystem::subscribe_safety()
{
  estop_sub_.reset();
  enable_sub_.reset();
  rclcpp::Node::SharedPtr node;
  try {
    node = get_node();
  } catch (const std::exception & ex) {
    RCLCPP_WARN(get_logger(), "No hardware node for e-stop yet: %s", ex.what());
    return;
  }
  if (!node) {
    RCLCPP_WARN(get_logger(), "Hardware node is null; /emergency_stop is not subscribed");
    return;
  }
  estop_sub_ = node->create_subscription<std_msgs::msg::Bool>(
    "/emergency_stop", rclcpp::QoS(10),
    [this](const std_msgs::msg::Bool & msg) {
      estop_.store(msg.data);
      if (msg.data) {
        arm_enable_.store(false);
      }
    });
  enable_sub_ = node->create_subscription<std_msgs::msg::Bool>(
    "/arm/enable", rclcpp::QoS(10),
    [this](const std_msgs::msg::Bool & msg) {
      if (estop_.load()) {
        arm_enable_.store(false);
        return;
      }
      arm_enable_.store(msg.data);
    });
}

hardware_interface::CallbackReturn OmniBotSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::string error;
  if (!base_port_.open_port(base_path_, base_baud_, error)) {
    RCLCPP_ERROR(get_logger(), "%s", error.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!arm_port_.open_port(arm_path_, arm_baud_, error)) {
    RCLCPP_ERROR(get_logger(), "%s", error.c_str());
    base_port_.close_port();
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!buses_->configure(error)) {
    RCLCPP_ERROR(get_logger(), "%s", error.c_str());
    close_ports();
    return hardware_interface::CallbackReturn::ERROR;
  }
  subscribe_safety();
  RCLCPP_INFO(get_logger(), "OmniBot ports open (%s, %s)", base_path_.c_str(), arm_path_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool OmniBotSystem::apply_measured_hold()
{
  try {
    for (int i = 0; i < 6; ++i) {
      const auto & name = arm_names_[static_cast<size_t>(i)];
      const double position = buses_->arm_position()[static_cast<size_t>(i)];
      const double velocity = buses_->arm_velocity()[static_cast<size_t>(i)];
      set_state(name + "/" + hardware_interface::HW_IF_POSITION, position);
      set_state(name + "/" + hardware_interface::HW_IF_VELOCITY, velocity);
      set_command(name + "/" + hardware_interface::HW_IF_POSITION, position);
    }
    for (int i = 0; i < 4; ++i) {
      const auto & name = wheel_names_[static_cast<size_t>(i)];
      set_state(name + "/" + hardware_interface::HW_IF_POSITION, buses_->wheel_position()[static_cast<size_t>(i)]);
      set_state(name + "/" + hardware_interface::HW_IF_VELOCITY, buses_->wheel_velocity()[static_cast<size_t>(i)]);
      set_command(name + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Could not seed hold command: %s", ex.what());
    return false;
  }
  return true;
}

hardware_interface::CallbackReturn OmniBotSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::string error;
  if (!buses_->capture_arm_hold(error)) {
    RCLCPP_ERROR(get_logger(), "%s", error.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (estop_.load()) {
    buses_->stop(error);
    arm_enable_.store(false);
    torque_written_on_ = false;
  } else {
    torque_written_on_ = true;
  }
  if (!apply_measured_hold()) {
    stop_motion();
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool OmniBotSystem::stop_motion()
{
  if (!buses_) {
    return true;
  }
  std::string error;
  const bool ok = buses_->stop(error);
  torque_written_on_ = false;
  arm_enable_.store(false);
  if (!ok) {
    RCLCPP_ERROR(get_logger(), "%s", error.c_str());
  }
  return ok;
}

hardware_interface::CallbackReturn OmniBotSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stop_motion();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OmniBotSystem::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stop_motion();
  return hardware_interface::CallbackReturn::SUCCESS;
}

void OmniBotSystem::close_ports()
{
  estop_sub_.reset();
  enable_sub_.reset();
  base_port_.close_port();
  arm_port_.close_port();
}

hardware_interface::CallbackReturn OmniBotSystem::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stop_motion();
  close_ports();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type OmniBotSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  buses_->poll_base(period.seconds());
  buses_->poll_arm();
  try {
    for (int i = 0; i < 4; ++i) {
      const auto & name = wheel_names_[static_cast<size_t>(i)];
      set_state(name + "/" + hardware_interface::HW_IF_POSITION, buses_->wheel_position()[static_cast<size_t>(i)]);
      set_state(name + "/" + hardware_interface::HW_IF_VELOCITY, buses_->wheel_velocity()[static_cast<size_t>(i)]);
    }
    for (int i = 0; i < 6; ++i) {
      const auto & name = arm_names_[static_cast<size_t>(i)];
      set_state(name + "/" + hardware_interface::HW_IF_POSITION, buses_->arm_position()[static_cast<size_t>(i)]);
      set_state(name + "/" + hardware_interface::HW_IF_VELOCITY, buses_->arm_velocity()[static_cast<size_t>(i)]);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OmniBotSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::string error;
  const bool stopped = estop_.load();
  const bool want_torque = arm_enable_.load() && !stopped;
  std::array<double, 4> wheels{};
  try {
    if (stopped) {
      wheels.fill(0.0);
    } else {
      for (int i = 0; i < 4; ++i) {
        wheels[static_cast<size_t>(i)] = get_command(
          wheel_names_[static_cast<size_t>(i)] + "/" + hardware_interface::HW_IF_VELOCITY);
      }
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
    return hardware_interface::return_type::ERROR;
  }
  if (!buses_->write_wheel_velocities(wheels, error)) {
    RCLCPP_ERROR(get_logger(), "%s", error.c_str());
    return hardware_interface::return_type::ERROR;
  }
  if (want_torque != torque_written_on_) {
    if (!buses_->set_arm_torque(want_torque, error)) {
      RCLCPP_ERROR(get_logger(), "%s", error.c_str());
      return hardware_interface::return_type::ERROR;
    }
    torque_written_on_ = want_torque;
  }
  if (want_torque) {
    std::array<double, 6> arm{};
    try {
      for (int i = 0; i < 6; ++i) {
        arm[static_cast<size_t>(i)] = get_command(
          arm_names_[static_cast<size_t>(i)] + "/" + hardware_interface::HW_IF_POSITION);
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "%s", ex.what());
      return hardware_interface::return_type::ERROR;
    }
    if (!buses_->write_arm_positions(arm, error)) {
      RCLCPP_ERROR(get_logger(), "%s", error.c_str());
      return hardware_interface::return_type::ERROR;
    }
  }
  if (!buses_->request_arm_feedback(error)) {
    RCLCPP_ERROR(get_logger(), "%s", error.c_str());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace omnibot_hardware

PLUGINLIB_EXPORT_CLASS(omnibot_hardware::OmniBotSystem, hardware_interface::SystemInterface)

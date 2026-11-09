// Copyright 2026 OhhO Robotics
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNIBOT_HARDWARE__OMNIBOT_SYSTEM_HPP_
#define OMNIBOT_HARDWARE__OMNIBOT_SYSTEM_HPP_

#include <array>
#include <atomic>
#include <memory>
#include <string>

#include "hardware_interface/system_interface.hpp"
#include "omnibot_hardware/hardware_buses.hpp"
#include "omnibot_hardware/serial_port.hpp"
#include "rclcpp/subscription.hpp"
#include "std_msgs/msg/bool.hpp"

namespace omnibot_hardware
{

class OmniBotSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void subscribe_safety();
  void close_ports();
  bool stop_motion();
  bool apply_measured_hold();

  LinuxSerialPort base_port_;
  LinuxSerialPort arm_port_;
  BusConfig config_;
  std::unique_ptr<HardwareBuses> buses_;
  std::array<std::string, 4> wheel_names_;
  std::array<std::string, 6> arm_names_;
  std::string base_path_;
  std::string arm_path_;
  int base_baud_{115200};
  int arm_baud_{1000000};
  std::atomic<bool> estop_{false};
  std::atomic<bool> arm_enable_{true};
  bool torque_written_on_{false};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
};

}  // namespace omnibot_hardware

#endif  // OMNIBOT_HARDWARE__OMNIBOT_SYSTEM_HPP_

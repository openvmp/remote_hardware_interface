/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-10-29
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_REMOTE_HARDWARE_INTERFACE_ACTUATOR_INTERFACE_H
#define OPENVMP_REMOTE_HARDWARE_INTERFACE_ACTUATOR_INTERFACE_H

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
// #include "ros2_control_demo_hardware/visibility_control.h"

namespace remote_hardware_interface {

class RemoteActuatorInterface : public hardware_interface::ActuatorInterface {
 public:
  // RCLCPP_SHARED_PTR_DEFINITIONS(RemoteActuatorInterface);

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;

  // hardware_interface::CallbackReturn on_activate(
  //     const rclcpp_lifecycle::State& previous_state) override;

  // hardware_interface::CallbackReturn on_deactivate(
  //     const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  // // Parameters for the simulation
  // double hw_start_sec_;
  // double hw_stop_sec_;
  // double hw_slowdown_;

  // // Store the command for the simulated robot
  // double hw_joint_command_;
  // double hw_joint_state_;
};

}  // namespace remote_hardware_interface

#endif  // OPENVMP_REMOTE_HARDWARE_INTERFACE_ACTUATOR_INTERFACE_H
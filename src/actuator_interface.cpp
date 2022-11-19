/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-10-29
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_hardware_interface/actuator_interface.hpp"

#include <exception>

namespace remote_hardware_interface {

hardware_interface::CallbackReturn RemoteActuatorInterface::on_init(
    const hardware_interface::HardwareInfo& actuator_info) {
  fprintf(stderr, "RemoteActuatorInterface: start\n");

  if (ActuatorInterface::on_init(actuator_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  fprintf(stderr, "RemoteActuatorInterface: %lu parameters\n",
          info_.hardware_parameters.size());
  for (const auto& hardware_parameter : info_.hardware_parameters) {
    fprintf(stderr, "RemoteActuatorInterface: %s = %s\n",
            hardware_parameter.first.c_str(),
            hardware_parameter.second.c_str());
  }

  fprintf(stderr, "RemoteActuatorInterface: %lu joints\n", info_.joints.size());
  for (const auto& joint : info_.joints) {
    fprintf(stderr, "RemoteActuatorInterface: joint %s\n", joint.name.c_str());
    for (const auto& state_interface : joint.state_interfaces) {
      fprintf(stderr, "RemoteActuatorInterface: state_interface %s (%s)\n",
              state_interface.name.c_str(), state_interface.data_type.c_str());
    }
    for (const auto& command_interface : joint.command_interfaces) {
      fprintf(stderr, "RemoteActuatorInterface: command_interfaces %s (%s)\n",
              command_interface.name.c_str(),
              command_interface.data_type.c_str());
    }
  }

  fprintf(stderr, "RemoteActuatorInterface: %lu transmissions\n",
          info_.transmissions.size());
  for (const auto& transmission : info_.transmissions) {
    fprintf(stderr, "RemoteActuatorInterface: transmission %s (%s)\n",
            transmission.name.c_str(), transmission.type.c_str());
    for (const auto& joint : transmission.joints) {
      fprintf(stderr, "RemoteActuatorInterface: transmission joint %s (%s)\n",
              joint.name.c_str(), joint.role.c_str());
      fprintf(stderr, "RemoteActuatorInterface: mechanical_reduction %f\n",
              joint.mechanical_reduction);
      fprintf(stderr, "RemoteActuatorInterface: offset %f\n", joint.offset);
      for (const auto& interface : joint.interfaces) {
        fprintf(stderr, "RemoteActuatorInterface: interface %s\n",
                interface.c_str());
      }
    }
    for (const auto& actuator : transmission.actuators) {
      fprintf(stderr,
              "RemoteActuatorInterface: transmission actuator %s (%s)\n",
              actuator.name.c_str(), actuator.role.c_str());
    }
  }

  // // can only control one joint
  // if (info_.joints.size() != 1) {
  //   return CallbackReturn::ERROR;
  // }
  // // can only control in position
  // const auto& command_interfaces = info_.joints[0].command_interfaces;
  // if (command_interfaces.size() != 1) {
  //   return CallbackReturn::ERROR;
  // }
  // if (command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
  //   return CallbackReturn::ERROR;
  // }
  // // can only give feedback state for position and velocity
  // const auto& state_interfaces = info_.joints[0].state_interfaces;
  // if (state_interfaces.size() < 1) {
  //   return CallbackReturn::ERROR;
  // }
  // for (const auto& state_interface : state_interfaces) {
  //   if ((state_interface.name != hardware_interface::HW_IF_POSITION) &&
  //       (state_interface.name != hardware_interface::HW_IF_VELOCITY)) {
  //     return CallbackReturn::ERROR;
  //   }
  // }
  fprintf(stderr, "RemoteActuatorInterface configured successfully.\n");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RemoteActuatorInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // const auto& joint_name = info_.joints[0].name;
  // state_interfaces.emplace_back(hardware_interface::StateInterface(
  //     joint_name, hardware_interface::HW_IF_POSITION, &position_state_));
  // state_interfaces.emplace_back(hardware_interface::StateInterface(
  //     joint_name, hardware_interface::HW_IF_VELOCITY, &velocity_state_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RemoteActuatorInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // const auto& joint_name = info_.joints[0].name;
  // command_interfaces.emplace_back(hardware_interface::CommandInterface(
  //     joint_name, hardware_interface::HW_IF_POSITION, &position_command_));

  return command_interfaces;
}

hardware_interface::return_type RemoteActuatorInterface::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RemoteActuatorInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // velocity_state_ = position_command_ - position_state_;
  // position_state_ = position_command_;
  return hardware_interface::return_type::OK;
}

}  // namespace remote_hardware_interface

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(remote_hardware_interface::RemoteActuatorInterface,
                       hardware_interface::ActuatorInterface)

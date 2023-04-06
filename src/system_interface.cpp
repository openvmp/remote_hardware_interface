/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-10-29
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_hardware_interface/system_interface.hpp"

#include <exception>

#include "rclcpp/node_options.hpp"
#include "remote_actuator/factory.hpp"
#include "remote_encoder/factory.hpp"

#ifndef DEBUG
#undef RCLCPP_DEBUG
#define RCLCPP_DEBUG(...)
#endif

namespace remote_hardware_interface {

hardware_interface::CallbackReturn RemoteSystemInterface::on_init(
    const hardware_interface::HardwareInfo& hardware_info) {
#ifdef DEBUG
  auto logger = rclcpp::get_logger("remote_hardware_interface");
#endif

  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  node_ = std::make_shared<rclcpp::Node>(
      "remote_hardware_interface",
      hardware_info.hardware_parameters.at("namespace"));
  executor_->add_node(node_);
  auto spin = [this]() {
    while (rclcpp::ok()) {
      executor_->spin_once();
    }
  };
  thread_executor_spin_ = std::thread(spin);

  RCLCPP_DEBUG(logger, "RemoteSystemInterface: start\n");

  if (SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  RCLCPP_DEBUG(logger, "RemoteSystemInterface: %lu parameters\n",
               info_.hardware_parameters.size());
#ifdef DEBUG
  for (const auto& hardware_parameter : info_.hardware_parameters) {
    RCLCPP_DEBUG(logger, "RemoteSystemInterface: %s = %s\n",
                 hardware_parameter.first.c_str(),
                 hardware_parameter.second.c_str());
  }
#endif

  RCLCPP_DEBUG(logger, "RemoteSystemInterface: %lu joints\n",
               info_.joints.size());
  for (const auto& joint : info_.joints) {
    RCLCPP_DEBUG(logger, "RemoteSystemInterface: joint %s\n",
                 joint.name.c_str());

    if (joint.command_interfaces.size() > 0) {
      auto actuator_client =
          remote_actuator::Factory::New(node_.get(), "/actuator/" + joint.name);

      for (const auto& command_interface : joint.command_interfaces) {
        RCLCPP_DEBUG(logger,
                     "RemoteSystemInterface: command_interface %s (%s)\n",
                     command_interface.name.c_str(),
                     command_interface.data_type.c_str());

        if (command_interface.name == hardware_interface::HW_IF_POSITION) {
          if (actuator_client->has_position()) {
            RCLCPP_DEBUG(
                logger,
                "RemoteSystemInterface: command_interface: has position\n");
            actuators_.insert(
                {joint.name + "/" + hardware_interface::HW_IF_POSITION,
                 actuator_client});
          } else {
            RCLCPP_DEBUG(
                logger,
                "The model requires a position command interface at %s but "
                "the driver does not provide it\n",
                joint.name.c_str());
          }
        } else if (command_interface.name ==
                   hardware_interface::HW_IF_VELOCITY) {
          if (actuator_client->has_velocity()) {
            RCLCPP_DEBUG(
                logger,
                "RemoteSystemInterface: command_interface: has velocity\n");
            actuators_.insert(
                {joint.name + "/" + hardware_interface::HW_IF_VELOCITY,
                 actuator_client});
          } else {
            RCLCPP_DEBUG(
                logger,
                "The model requires a velocity command interface at %s but "
                "the driver does not provide it\n",
                joint.name.c_str());
          }
        }
      }
    }

    if (joint.state_interfaces.size() > 0) {
      auto encoder_client =
          remote_encoder::Factory::New(node_.get(), "/encoder/" + joint.name);

      for (const auto& state_interface : joint.state_interfaces) {
        RCLCPP_DEBUG(logger, "RemoteSystemInterface: state_interface %s (%s)\n",
                     state_interface.name.c_str(),
                     state_interface.data_type.c_str());

        if (state_interface.name == hardware_interface::HW_IF_POSITION) {
          if (encoder_client->has_position()) {
            RCLCPP_DEBUG(
                logger,
                "RemoteSystemInterface: state_interface: has position\n");
            encoders_.insert(
                {joint.name + "/" + hardware_interface::HW_IF_POSITION,
                 encoder_client});
          } else {
            RCLCPP_DEBUG(
                logger,
                "The model requires a position state interface at %s but "
                "the driver does not provide it\n",
                joint.name.c_str());
          }
        } else if (state_interface.name == hardware_interface::HW_IF_VELOCITY) {
          if (encoder_client->has_velocity()) {
            RCLCPP_DEBUG(
                logger,
                "RemoteSystemInterface: state_interface: has velocity\n");
            encoders_.insert(
                {joint.name + "/" + hardware_interface::HW_IF_VELOCITY,
                 encoder_client});
          } else {
            RCLCPP_DEBUG(
                logger,
                "The model requires a velocity state interface at %s but "
                "the driver does not provide it\n",
                joint.name.c_str());
          }
        }
      }
    }
  }

  RCLCPP_DEBUG(logger, "RemoteSystemInterface: %lu transmissions\n",
               info_.transmissions.size());
#ifdef DEBUG
  for (const auto& transmission : info_.transmissions) {
    RCLCPP_DEBUG(logger, "RemoteSystemInterface: transmission %s (%s)\n",
                 transmission.name.c_str(), transmission.type.c_str());
    for (const auto& joint : transmission.joints) {
      RCLCPP_DEBUG(logger,
                   "RemoteSystemInterface: transmission joint %s (%s)\n",
                   joint.name.c_str(), joint.role.c_str());
      RCLCPP_DEBUG(logger, "RemoteSystemInterface: mechanical_reduction %f\n",
                   joint.mechanical_reduction);
      RCLCPP_DEBUG(logger, "RemoteSystemInterface: offset %f\n", joint.offset);
      for (const auto& interface : joint.interfaces) {
        RCLCPP_DEBUG(logger, "RemoteSystemInterface: interface %s\n",
                     interface.c_str());
      }
    }
    for (const auto& actuator : transmission.actuators) {
      RCLCPP_DEBUG(logger,
                   "RemoteSystemInterface: transmission actuator %s (%s)\n",
                   actuator.name.c_str(), actuator.role.c_str());
    }
  }
#endif

  RCLCPP_DEBUG(logger, "RemoteSystemInterface configured successfully.\n");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RemoteSystemInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (const auto& joint : info_.joints) {
    for (const auto& state_interface : joint.state_interfaces) {
      auto clnt_it = encoders_.find(joint.name + "/" + state_interface.name);
      if (clnt_it != encoders_.end()) {
        auto intf = clnt_it->second;
        auto clnt = std::make_shared<EncoderClient>(intf, state_interface.name);
        client_encoders_.push_back(clnt);

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint.name, state_interface.name, &clnt->state));
      }
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RemoteSystemInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (const auto& joint : info_.joints) {
    for (const auto& command_interface : joint.command_interfaces) {
      auto clnt_it = actuators_.find(joint.name + "/" + command_interface.name);
      if (clnt_it != actuators_.end()) {
        auto intf = clnt_it->second;
        auto clnt =
            std::make_shared<ActuatorClient>(intf, command_interface.name);
        client_actuators_.push_back(clnt);

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint.name, command_interface.name, &clnt->command));
      }
    }
  }

  return command_interfaces;
}

hardware_interface::return_type RemoteSystemInterface::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  for (const auto& encoder : client_encoders_) {
    encoder->read();
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RemoteSystemInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // velocity_state_ = position_command_ - position_state_;
  // position_state_ = position_command_;

  for (const auto& actuator : client_actuators_) {
    actuator->write();
  }

  return hardware_interface::return_type::OK;
}

}  // namespace remote_hardware_interface

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(remote_hardware_interface::RemoteSystemInterface,
                       hardware_interface::SystemInterface)

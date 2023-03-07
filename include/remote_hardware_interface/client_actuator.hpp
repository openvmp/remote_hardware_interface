/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-04
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_REMOTE_HARDWARE_INTERFACE_CLIENT_ACTUATOR_H
#define OPENVMP_REMOTE_HARDWARE_INTERFACE_CLIENT_ACTUATOR_H

#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "remote_actuator/interface.hpp"

namespace remote_hardware_interface {

class ActuatorClient final {
 public:
  ActuatorClient(std::shared_ptr<remote_actuator::Interface>,
                 const std::string &);

  double command;
  void write();

 private:
  std::shared_ptr<remote_actuator::Interface> prov_;
  std::function<void(double)> write_func_;
};

}  // namespace remote_hardware_interface

#endif  // OPENVMP_REMOTE_HARDWARE_INTERFACE_CLIENT_ACTUATOR_H
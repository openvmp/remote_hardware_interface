/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-04
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_hardware_interface/client_actuator.hpp"

#include <memory>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "remote_actuator/factory.hpp"

namespace remote_hardware_interface {

ActuatorClient::ActuatorClient(std::shared_ptr<remote_actuator::Interface> prov,
                               const std::string &name)
    : command{0.0}, prov_{prov} {
  if (name == hardware_interface::HW_IF_POSITION) {
    write_func_ = std::bind(&remote_actuator::Interface::position_set,
                            prov_.get(), std::placeholders::_1);
  } else if (name == hardware_interface::HW_IF_VELOCITY) {
    write_func_ = std::bind(&remote_actuator::Interface::velocity_set,
                            prov_.get(), std::placeholders::_1);
  }
}

void ActuatorClient::write() {
  // trigger the corresponding setter
  write_func_(command);
}

}  // namespace remote_hardware_interface
/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-05
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_hardware_interface/client_encoder.hpp"

#include <memory>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "remote_encoder/factory.hpp"

namespace remote_hardware_interface {

EncoderClient::EncoderClient(std::shared_ptr<remote_encoder::Interface> prov,
                             const std::string &name)
    : state{0.0}, prov_{prov} {
  if (name == hardware_interface::HW_IF_POSITION) {
    read_func_ =
        std::bind(&remote_encoder::Interface::position_get, prov_.get());
  } else if (name == hardware_interface::HW_IF_VELOCITY) {
    read_func_ =
        std::bind(&remote_encoder::Interface::velocity_get, prov_.get());
  }
}

void EncoderClient::read() {
  // trigger the corresponding setter
  state = read_func_();
}

}  // namespace remote_hardware_interface

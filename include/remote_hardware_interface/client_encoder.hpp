/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-04
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_REMOTE_HARDWARE_INTERFACE_CLIENT_ENCODER_H
#define OPENVMP_REMOTE_HARDWARE_INTERFACE_CLIENT_ENCODER_H

#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "remote_encoder/interface.hpp"

namespace remote_hardware_interface {

class EncoderClient final {
 public:
  EncoderClient(std::shared_ptr<remote_encoder::Interface>,
                const std::string &);

  double state;
  void read();

 private:
  std::shared_ptr<remote_encoder::Interface> prov_;
  std::function<double()> read_func_;
};

}  // namespace remote_hardware_interface

#endif  // OPENVMP_REMOTE_HARDWARE_INTERFACE_CLIENT_ENCODER_H
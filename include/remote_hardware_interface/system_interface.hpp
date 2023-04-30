/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-10-29
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_REMOTE_HARDWARE_INTERFACE_SYSTEM_INTERFACE_H
#define OPENVMP_REMOTE_HARDWARE_INTERFACE_SYSTEM_INTERFACE_H

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

#include "remote_actuator/interface.hpp"
#include "remote_hardware_interface/client_actuator.hpp"
#include "remote_hardware_interface/client_encoder.hpp"

namespace remote_hardware_interface {

class RemoteSystemInterface final : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RemoteSystemInterface)

  virtual ~RemoteSystemInterface();

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo &info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;

  // hardware_interface::CallbackReturn on_activate(
  //     const rclcpp_lifecycle::State& previous_state) override;

  // hardware_interface::CallbackReturn on_deactivate(
  //     const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;

  hardware_interface::return_type write(
      const rclcpp::Time &time, const rclcpp::Duration &period) override;

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
  std::thread thread_executor_spin_;
  std::vector<std::thread> workers_;
  volatile bool workers_do_stop_;

  class Task {
   public:
    std::function<void()> func;
    std::mutex mutex;
    std::unique_lock<std::mutex> lock;
    std::condition_variable cv;
  };

  std::list<std::shared_ptr<Task>> tasks_;
  std::mutex tasks_mutex_;
  std::condition_variable tasks_mutex_condition_;
  // // Parameters for the simulation
  // double hw_start_sec_;
  // double hw_stop_sec_;
  // double hw_slowdown_;

  // // Store the command for the simulated robot
  // double hw_joint_command_;
  // double hw_joint_state_;

  std::vector<std::shared_ptr<ActuatorClient>> client_actuators_;
  std::vector<std::shared_ptr<EncoderClient>> client_encoders_;
  std::map<std::string, std::shared_ptr<remote_actuator::Interface>> actuators_;
  std::map<std::string, std::shared_ptr<remote_encoder::Interface>> encoders_;

  void worker_loop_();
};

}  // namespace remote_hardware_interface

#endif  // OPENVMP_REMOTE_HARDWARE_INTERFACE_SYSTEM_INTERFACE_H

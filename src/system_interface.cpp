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

#define MAX_ENCODER_TASKS_SCHEDULED 50

#ifndef DEBUG
#undef RCLCPP_DEBUG
#if 1
#define RCLCPP_DEBUG(...)
#else
#define DEBUG
#define RCLCPP_DEBUG RCLCPP_INFO
#endif
#endif

namespace remote_hardware_interface {

hardware_interface::CallbackReturn RemoteSystemInterface::on_init(
    const hardware_interface::HardwareInfo& hardware_info) {
  RCLCPP_DEBUG(node_->get_logger(), "RemoteSystemInterface: start");

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

  RCLCPP_DEBUG(node_->get_logger(), "RemoteSystemInterface: node started");

  if (SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  RCLCPP_DEBUG(node_->get_logger(), "RemoteSystemInterface: %lu parameters",
               info_.hardware_parameters.size());
#ifdef DEBUG
  for (const auto& hardware_parameter : info_.hardware_parameters) {
    RCLCPP_DEBUG(node_->get_logger(), "RemoteSystemInterface: %s = %s",
                 hardware_parameter.first.c_str(),
                 hardware_parameter.second.c_str());
  }
#endif

  RCLCPP_DEBUG(node_->get_logger(), "RemoteSystemInterface: %lu joints",
               info_.joints.size());
  for (const auto& joint : info_.joints) {
    RCLCPP_DEBUG(node_->get_logger(), "RemoteSystemInterface: joint %s",
                 joint.name.c_str());

    if (joint.command_interfaces.size() > 0) {
      auto actuator_client =
          remote_actuator::Factory::New(node_.get(), "/actuator/" + joint.name);

      for (const auto& command_interface : joint.command_interfaces) {
        RCLCPP_DEBUG(node_->get_logger(),
                     "RemoteSystemInterface: command_interface %s (%s)",
                     command_interface.name.c_str(),
                     command_interface.data_type.c_str());

        if (command_interface.name == hardware_interface::HW_IF_POSITION) {
          if (actuator_client->has_position()) {
            RCLCPP_DEBUG(
                node_->get_logger(),
                "RemoteSystemInterface: command_interface: has position");
            actuators_.insert(
                {joint.name + "/" + hardware_interface::HW_IF_POSITION,
                 actuator_client});
          } else {
            RCLCPP_DEBUG(
                node_->get_logger(),
                "The model requires a position command interface at %s but "
                "the driver does not provide it",
                joint.name.c_str());
          }
        } else if (command_interface.name ==
                   hardware_interface::HW_IF_VELOCITY) {
          if (actuator_client->has_velocity()) {
            RCLCPP_DEBUG(
                node_->get_logger(),
                "RemoteSystemInterface: command_interface: has velocity");
            actuators_.insert(
                {joint.name + "/" + hardware_interface::HW_IF_VELOCITY,
                 actuator_client});
          } else {
            RCLCPP_DEBUG(
                node_->get_logger(),
                "The model requires a velocity command interface at %s but "
                "the driver does not provide it",
                joint.name.c_str());
          }
        }
      }
    }

    if (joint.state_interfaces.size() > 0) {
      auto encoder_client =
          remote_encoder::Factory::New(node_.get(), "/encoder/" + joint.name);

      for (const auto& state_interface : joint.state_interfaces) {
        RCLCPP_DEBUG(node_->get_logger(),
                     "RemoteSystemInterface: state_interface %s (%s)",
                     state_interface.name.c_str(),
                     state_interface.data_type.c_str());

        if (state_interface.name == hardware_interface::HW_IF_POSITION) {
          if (encoder_client->has_position()) {
            RCLCPP_DEBUG(
                node_->get_logger(),
                "RemoteSystemInterface: state_interface: has position");
            encoders_.insert(
                {joint.name + "/" + hardware_interface::HW_IF_POSITION,
                 encoder_client});
          } else {
            RCLCPP_DEBUG(
                node_->get_logger(),
                "The model requires a position state interface at %s but "
                "the driver does not provide it",
                joint.name.c_str());
          }
        } else if (state_interface.name == hardware_interface::HW_IF_VELOCITY) {
          if (encoder_client->has_velocity()) {
            RCLCPP_DEBUG(
                node_->get_logger(),
                "RemoteSystemInterface: state_interface: has velocity");
            encoders_.insert(
                {joint.name + "/" + hardware_interface::HW_IF_VELOCITY,
                 encoder_client});
          } else {
            RCLCPP_DEBUG(
                node_->get_logger(),
                "The model requires a velocity state interface at %s but "
                "the driver does not provide it",
                joint.name.c_str());
          }
        }
      }
    }
  }

  RCLCPP_DEBUG(node_->get_logger(), "RemoteSystemInterface: %lu transmissions",
               info_.transmissions.size());
#ifdef DEBUG
  for (const auto& transmission : info_.transmissions) {
    RCLCPP_DEBUG(node_->get_logger(),
                 "RemoteSystemInterface: transmission %s (%s)",
                 transmission.name.c_str(), transmission.type.c_str());
    for (const auto& joint : transmission.joints) {
      RCLCPP_DEBUG(node_->get_logger(),
                   "RemoteSystemInterface: transmission joint %s (%s)",
                   joint.name.c_str(), joint.role.c_str());
      RCLCPP_DEBUG(node_->get_logger(),
                   "RemoteSystemInterface: mechanical_reduction %f",
                   joint.mechanical_reduction);
      RCLCPP_DEBUG(node_->get_logger(), "RemoteSystemInterface: offset %f",
                   joint.offset);
      for (const auto& interface : joint.interfaces) {
        RCLCPP_DEBUG(node_->get_logger(), "RemoteSystemInterface: interface %s",
                     interface.c_str());
      }
    }
    for (const auto& actuator : transmission.actuators) {
      RCLCPP_DEBUG(node_->get_logger(),
                   "RemoteSystemInterface: transmission actuator %s (%s)",
                   actuator.name.c_str(), actuator.role.c_str());
    }
  }
#endif

  workers_do_stop_ = false;

  // Add one spare worker just in case
  workers_.emplace_back(std::bind(&RemoteSystemInterface::worker_loop_, this));

  RCLCPP_DEBUG(node_->get_logger(),
               "RemoteSystemInterface configured successfully.");
  return CallbackReturn::SUCCESS;
}

RemoteSystemInterface::~RemoteSystemInterface() {
  workers_do_stop_ = true;
  for (auto& worker : workers_) {
    worker.join();
  }
}

void RemoteSystemInterface::worker_loop_() {
  while (!workers_do_stop_) {
    std::shared_ptr<Task> task;

    std::unique_lock<std::mutex> lock(tasks_mutex_);
    tasks_mutex_condition_.wait(
        lock, [this] { return !tasks_.empty() || workers_do_stop_; });
    if (workers_do_stop_) {
      break;
    }

    if (!tasks_.empty()) {
      task = tasks_.front();
      tasks_.pop_front();
    }
    RCLCPP_DEBUG(node_->get_logger(), "tasks left: %d", (int)tasks_.size());
    lock.unlock();

    if (task) {
      task->func();
      task->cv.notify_one();
    }
  }
  RCLCPP_DEBUG(node_->get_logger(), "worker stopped");
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

        workers_.emplace_back(
            std::bind(&RemoteSystemInterface::worker_loop_, this));
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

        workers_.emplace_back(
            std::bind(&RemoteSystemInterface::worker_loop_, this));
      }
    }
  }

  return command_interfaces;
}

hardware_interface::return_type RemoteSystemInterface::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  std::vector<std::shared_ptr<Task>> tasks;

  tasks_mutex_.lock();
  if (tasks_.size() > MAX_ENCODER_TASKS_SCHEDULED) {
    tasks_mutex_.unlock();
    RCLCPP_INFO(node_->get_logger(), "tasks queued: %d", (int)tasks_.size());
    // we are going to miss some stuff anyway
    return hardware_interface::return_type::OK;
  }
  tasks_mutex_.unlock();

  for (const auto& encoder : client_encoders_) {
    auto task = std::make_shared<Task>();
    task->func = [&encoder]() { encoder->read(); };
    task->lock = std::unique_lock<std::mutex>(task->mutex);

    auto tasks_lock = std::unique_lock<std::mutex>(tasks_mutex_);
    tasks_.push_back(task);
    tasks_mutex_condition_.notify_one();
  }

  for (auto& task : tasks) {
    task->cv.wait(task->lock);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RemoteSystemInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  std::vector<std::shared_ptr<Task>> tasks;

  for (const auto& actuator : client_actuators_) {
    auto task = std::make_shared<Task>();
    task->func = [&actuator]() { actuator->write(); };
    task->lock = std::unique_lock<std::mutex>(task->mutex);

    auto tasks_lock = std::unique_lock<std::mutex>(tasks_mutex_);
    tasks_.push_back(task);
    tasks_mutex_condition_.notify_one();
  }

  for (auto& task : tasks) {
    task->cv.wait(task->lock);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace remote_hardware_interface

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(remote_hardware_interface::RemoteSystemInterface,
                       hardware_interface::SystemInterface)

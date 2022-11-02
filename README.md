# OpenVMP

[![License](./license.svg)](./LICENSE.txt)

This package is a part of [the OpenVMP project](https://github.com/openvmp/openvmp).
But it's designed to be universal and usable independently from the rest of OpenVMP or in a combination with select OpenVMP packages.

## remote\_motion\_hardware

This package implements a hardware interface plugin for ros2\_control
and proxies requests to compatible nodes that actually implement the hardware
control.

This is useful to access remote hardware (over the network) or to access
heavy hardware modules that are not designed for an exclusive use by
the ros2\_control\_node (e.g. for independent power consumption control,
troubleshooting/introspection, audit log recording,
or other non-motion-control related operations).
Examples of such hardware modules include:

- Stepper motor drivers that have ROS2 packages based on the
  [stepper\_driver](https://github.com/openvmp/stepper_driver/) interface:
  - [stepper\_driver\_em2rs](https://github.com/openvmp/stepper_driver_em2rs/)
- Electromagnetic brakes controlled by relay boards compatible with:
  - [R413D08](https://github.com/openvmp/switch_r413d08/)


### How to

#### Configuration

Add the following to the URDF files:

```
  <hardware>
    <plugin>remote_motion_hardware/ActuatorInterface</plugin>
    <param name="prefix">/robot1</param>
  </hardware>
```

- prefix: the prefix to ROS2 interfaces

  E.g. if the prefix is "/robot1" then the actuator of the join "front" will
  be accessed at "/robot1/front".

#### Known limitations

- Only the actuator interface is supported for now

### Implementation details

```mermaid
flowchart TB
    urdf[URDF]

    subgraph ros2_control_node[Process: ros2_control_node]
      controller_manager
      subgraph remote_hardware_interface[Controller: remote_hardware_interface]
        subgraph joint1[ActuatorInterface: right_shoulder_joint]
          subgraph state_interface[state_interface]
            absolute_encoder_intf[Absolute encoder]
          end
          subgraph command_interface[command_interface]
            stepper_driver_intf[Stepper driver]
            brake_intf[Electromagnetic brake]
          end
        end
      end
      controller_manager --> remote_hardware_interface
    end
    
    subgraph stepper_driver_node[Example stepper driver node]
      stepper_driver_impl[//stepper_driver/motor1/] -->
      stepper_logic[Stepper driver's driver] -->
      stepper_driver_topic[//modbus/line1/02/]
    end
    
    subgraph absolute_encoder_node[Example stepper driver node]
      absolute_encoder_impl[//encoder/angle1/] -->
      absolute_encoder_logic[Absolute encoder driver] -->
      absolute_encoder_topic[//modbus/line1/01/]
    end
    
    subgraph switch_node[Example switch node]
      switch_impl[//switch/brake1/] -->
      switch_driver[Switch driver] -->
      gpio_intf[GPIO driver]
    end

    subgraph modbus_rtu_node[Modbus RTU]
      modbus[//modbus/line1/] -->
      serial_driver["Serial port driver"]
    end

    absolute_encoder_intf --> absolute_encoder_impl
    stepper_driver_intf --> stepper_driver_impl

    absolute_encoder_topic --> modbus
    stepper_driver_topic --> modbus
    brake_intf --> switch_impl

    pmu[Example alternative consumer\nof hardware interfaces:\nPower Management Unit]
    pmu -->stepper_driver_impl
    pmu -->switch_impl

    gpio_intf ----> gpio{{GPIO interface}} --> relay

    urdf --> controller_manager
    subgraph robot_hardware[Example robot hardware]
      rs485[RS485]
      rs485 --> encoder[Absolute encoder]
      rs485 --> stepper_driver[Stepper driver]
      relay[Relay] --> brake[Elecromagnetic brake]
    end

    serial_driver --> file{{Character device: /dev/ttyS0}} --> rs485

    style ros2_control_node fill:#0000
    style stepper_driver_node fill:#0000
    style absolute_encoder_node fill:#0000
    style switch_node fill:#0000
    style modbus_rtu_node fill:#0000
    style robot_hardware fill:#0000
```

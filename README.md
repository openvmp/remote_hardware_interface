# remote\_hardware\_interface

[![License](./apache20.svg)](./LICENSE.txt)

This package implements a hardware interface plugin for ros2\_control
and proxies requests to compatible nodes that actually implement the hardware
control.

This is useful to access remote hardware (over the network) or to access
heavy hardware modules that are not designed for an exclusive use by
the ros2\_control\_node (e.g. for independent power consumption control,
troubleshooting/introspection, audit log recording,
or other non-motion-control related operations).
Examples of such hardware modules include:

- Stepper motor drivers:
  - [Leadshine/STEPPERONLINE Modbus](https://github.com/openvmp/stepper_driver_em2rs/)
  - PUL/DIR interfaces on [remote_microcontroller](https://github.com/openvmp/microcontroller/) (e.g. Arduino)
- Servo drivers:
  - PWM interfaces on [remote_microcontroller](https://github.com/openvmp/microcontroller/) (e.g. Arduino)
- Absolute encoder drivers:
  - [CUI Devices AMT21](https://github.com/openvmp/encoder_amt21/)


## How to

### Configuration

Here is an example of how it could look like in your URDF files:

```xml
  <ros2_control name="HardwareSystem" type="system">
    <hardware>
      <plugin>remote_hardware_interface/RemoteSystemInterface</plugin>
      <param name="namespace">/robot1</param>
    </hardware>
  </ros2_control>
```

- namespace: the prefix to all ROS2 interfaces
  - there will be a node created in this namespace to facilitate messaging with actuators and encoders
  - actuators will be expected to be at `<namespace>/actuator/<joint-name>`
  - encoders will be expected to be at `<namespace>/encoder/<joint-name>`

## Implementation details

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
      stepper_driver_impl[//actuator/joint1/] -->
      stepper_logic[Stepper driver's driver] -->
      stepper_driver_topic[//modbus/line1/02/]
    end
    
    subgraph absolute_encoder_node[Example stepper driver node]
      absolute_encoder_impl[//encoder/joint1/] -->
      absolute_encoder_logic[Absolute encoder driver] -->
      absolute_encoder_topic[//modbus/line1/01/]
    end
    
    subgraph switch_node[Example switch node]
      switch_impl[//brake/joint1/] -->
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

    pmu[Power management driver,\nas an example of alternative consumer\nof hardware interfaces]
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

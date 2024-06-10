# ROS2 Package for ODrive

This repository contains a ROS2 node intended for communication with ODrive motor controllers via CAN bus.

For information about installation, prerequisites, and getting started, checkout the ODrive [ROS CAN Package Guide](https://docs.odriverobotics.com/v/latest/guides/ros-package.html).

Compatible Devices:

- [ODrive Pro](https://odriverobotics.com/shop/odrive-pro)
- [ODrive S1](https://odriverobotics.com/shop/odrive-s1)
- [ODrive Micro](https://odriverobotics.com/shop/odrive-micro)

(not compatible with ODrive 3.x)

System Requirements:

- Ubuntu >= 20.04
- ROS2 >= Humble


## Interface

### Parameters

* `node_id`: The node_id of the device this node will attach to
* `interface`: the network interface name for the can bus

### Subscribes to

* `/control_message`: Input setpoints for the ODrive.

  The ODrive will interpret the values of input_pos, input_vel and input_torque depending on the control mode. 

  For example: In velocity control mode (2) input_pos is ignored, and input_torque is used as a feedforward term.

  **Note:** When changing `input_mode` or `control_mode`, it is advised to set the ODrive to IDLE before doing so. Changing these values during CLOSED_LOOP_CONTROL is not advised.

### Publishes

* `/odrive_status`: Provides ODrive/system level status updates.

  For this topic to work, the ODrive must be configured with the following [cyclic messages](https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#cyclic-messages) enabled:

  - `error_msg_rate_ms`
  - `temperature_msg_rate_ms`
  - `bus_voltage_msg_rate_ms`

  The ROS node will wait until one of each of these CAN messages has arrived before it emits a message on the `odrive_status` topic. Therefore, the largest period set here will dictate the period of the ROS2 message as well.

* `/controller_status`: Provides Controller level status updates. 

  For this topic to work, the ODrive must be configured with the following [cyclic messages](https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#cyclic-messages) enabled:

  - `heartbeat_msg_rate_ms`
  - `encoder_msg_rate_ms`
  - `iq_msg_rate_ms`
  - `torques_msg_rate_ms`

  The ROS node will wait until one of each of these CAN messages has arrived before it emits a message on the `controller_status` topic. Therefore, the largest period set here will dictate the period of the ROS2 message as well.

### Services

* `/request_axis_state`: Sets the axes requested state.

  This service requires regular heartbeat messages from the ODrive to determine the procedure result and will block until the procedure completes, with a minimum call time of 1 second.

### Data Types

All of the Message/Service fields are directly related to their corresponding CAN message. For more detailed information about each type, and how to interpet the data, please refer to the [ODrive CAN protocol documentation](https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#messages).

## Using Python Enums

**Python Example Node coming soon!**

In the meantime, here is how you can use the [odrive python package](https://pypi.org/project/odrive/) to display the enums:

    from odrive_can.msgs import ControllerStatus # remember to include odrive_can as a package dependency
    from odrive.enums import ProcedureResult
    ... # Node setup
    ctrl_stat = ControllerStatus()
    ... # receive message data
    print(ProcedureResult(ctrl_stat.procedure_result))


## Developer Notes

For user instructions, see [this guide](https://docs.odriverobotics.com/v/latest/guides/ros-package.html) instead.

You can build this node on a non-ROS developer PC by using the DevContainer configurations in this repository. For example with VSCode:

1. Clone repository
2. Open the repository folder in VSCode. It should automatically present an option "Reopen in Dev Container". Click on that and select the desired ROS version.
3. Once it's re-opened, in the VSCode terminal, run:

   ```
   colcon build --packages-select odrive_can
   source ./install/setup.bash
   ```
4. Running the node requires hardware access and only works if the container host is Linux.

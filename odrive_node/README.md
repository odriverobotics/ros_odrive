# Standalone ODrive ROS2 node

This packages serves as a standalone ROS2 node to control the ODrive via CAN.

For information about installation, prerequisites and getting started, check out the ODrive [ROS CAN Package Guide](https://docs.odriverobotics.com/v/latest/guides/ros-package.html).

## Interface

### Parameters

* `node_id`: The node_id of the device this node will attach to
* `interface`: the network interface name for the can bus
* `axis_idle_on_shutdown`: Whether to set ODrive to IDLE state when the node is terminated

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

  If the requested state is anything other than IDLE, this sends a `clear_errors` request to the ODrive (see below) before sending the state request.

* `/clear_errors`: Manual service call to clear disarm_reason and procedure_result, reset the LED color and re-arm the brake resistor if applicable. See also [`clear_errors()`](https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.clear_errors).

  This does not affect the axis state.

  If the axis dropped into IDLE because of an error and the intent is to re-enable it, call `/request_axis_state`
  instead with CLOSED_LOOP_CONTROL, which clears errors automatically.

### Data Types

All of the Message/Service fields are directly related to their corresponding CAN message. For more detailed information about each type, and how to interpet the data, please refer to the [ODrive CAN protocol documentation](https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#messages).

Enum types and their corresponding integer values are listed in the ODrive API reference:

- [AxisState](https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.Axis.AxisState)
- [ControlMode](https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.Controller.ControlMode)
- [InputMode](https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.Controller.InputMode)

If you have the `odrive` Python package installed (not mandatory for this ROS node), you translate enums like this:

```py
from odrive.enums import AxisState
print(AxisState.CLOSED_LOOP_CONTROL) # 8
print(AxisState(8).name) # CLOSED_LOOP_CONTROL
```

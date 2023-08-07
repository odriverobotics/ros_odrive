# ROS2 Package for ODrive Pro/S1 

This repository contains the development of a ROS2 node intended for communication with ODrive Pro/S1 devices over the CAN interface.

## Interface
### Nodes
#### odrive_can_node

This node is designed to interface with the ODrive Pro/S1 series of motor controllers over the CAN bus.

* subscribes to: /control_message (ControlMessage)

* publishes: 
    * /odrive_status (ODriveStatus)
    * /controller_status (ControllerStatus)

* service: /request_axis_state (AxisState)

* parameters:

    * node_id: The node_id of the device this node will attach to
    * interface: the network interface name for the can bus

### Messages

#### ControlMessage

Exposes the minimal set of inputs to use an ODrive. 

**important** Make sure the ODrive returns to IDLE before changing the input_mode, changing this value during CLOSED_LOOP_CONTROL is not advised.

The ODrive will interpret the values of input_pos, input_vel and input_torque depending on the control mode. 

For example: In velocity control mode (2) input_pos is ignored, and input_torque is used as a feedforward term.

#### ODriveStatus

Provides ODrive/system level status updates. 
Requires setting a non-zero period for the following [cyclic messages](https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#cyclic-messages)

`error_msg_rate_ms`
`temperature_msg_rate_ms`
`bus_voltage_msg_rate_ms`

**note** Each published message will wait for a new value from these endpoints. Therefore, the largest period set here will dictate the period of the ROS2 message as well.

#### ControllerStatus

Provides Controller level status updates. 
Requires setting a non-zero period for the following [cyclic messages](https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#cyclic-messages)

`heartbeat_msg_rate_ms`
`encoder_msg_rate_ms`
`iq_msg_rate_ms`
`torques_msg_rate_ms`

**note** Each published message will wait for a new value from these endpoints. Therefore, the largest period set here will dictate the period of the ROS2 message as well.

### Services

#### AxisState

Sets the axes requested state. This service requires regular heartbeat messages to determine the procedure result and will block until the procedure completes, with a minimum call time of 1 second.

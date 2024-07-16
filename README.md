# ROS2 Package for ODrive

This repository contains a ROS2 node intended for communication with ODrive motor controllers via CAN bus.

## Changes from the base package ##

We've made some changes to this package to make it more versatile than the base odrive_can package. They are defined below before the original Odrive readme.  

## New Publisher ##

### ODriveStatusAdvanced ###

We've added a custom ROS2 message type (i.e. an interface) that we are publishing called ODriveStatusAdvanced for facilitating sending out information on the Odrive and the Controller wihtout modifying the ControllerStatus or ODriveStatus publishers.

The structure of ODriveStatusAdvanced is shown below:
  
float32 bus_voltage  
float32 bus_current  
float32 fet_temperature  
float32 motor_temperature  
uint32 active_errors  
uint32 disarm_reason  
float32 pos_estimate  
float32 vel_estimate  
float32 torque_target  
float32 torque_estimate  
float32 iq_setpoint  
float32 iq_measured  
uint32 ctrl_active_errors  
uint8 axis_state  
uint8 procedure_result  
bool trajectory_done_flag  
float32 electrical_power  
float32 mechanical_power


This is a message that will be published from the Odrive consistently when data is available. 

The message consists of all the fields from ODriveStatus followed by all the fields from ControllerStatus. When the node receives status data back from the CAN bus it populates the ODriveStatus message, the ControllerStatus message and now it will populate the ODriveStatusAdvanced message using that same information and will publish that message when it has all the necessary data for each message.

[Important] ODriveStatusAdvanced sends out the electrical and mechanical power of the ODrive however, the powers_msg_rate_ms property on the ODrive is set to 0 by default. If this is left at 0 the ODrive won't send the powers over can and ODriveStatusAdvanced won't send. 

The topic of the ODriveStatusAdvanced publisher is "odrive_status_advanced". Though it should be noted that because of how the odrive_node works that in practice the topic will be the odrive's namespace followed by "odrive_status_advanced". E.g. "/pitch/odrv1/odrive_status_advanced".
  

## New Subscribers ##

### Reboot ###

We've added a custom ROS2 message called ControlPositionGain to take care of setting the Position gain via messages. 

It's structure is shown below: 

uint8 action

This is a message that will be sent to the Odrive.

Once the message is received by the Odrive's subscriber it will send a CAN message to reboot the odrive according to the action sent.

action - specifies the action to reboot with. The 2 supported modes are: 
    0 - reboot and reset setting to default  
    1 - reboot and save current configuration.   
  
[Note] The CAN side allows 4 potential actions which is why we left this as a uint8. The other 2 are: 
  3 - reboot and erase configuration. We haven't implemented this because this could do a lot of damage if improperly used.  
  4 - reboot and enter_dfu_mode2(). This puts the odrive into a configuration that allows it to update its firmware over CAN bus. We haven't implemented this because we can update over usb.  

The topic of the ControlGains subscriber is set to "control_pos_gain". Though it should be noted that because of how the odrive_node works that in practice the topic will be the odrive's namespace followed by "control_pos_gain". E.g. "/pitch/odrv1/control_pos_gain".


## Control Velocity Gains ##

We've added a custom ROS2 message called ControlVelocityGains to take care of setting the Velocity and Velocity Integrator gains as well as their limits via messages. 

It's structure is shown below: 

float32 vel_gain  
float32 vel_integrator_gain
float32 vel_integrator_limit
float32 vel_limit

This is a message that will be sent to the Odrive.

Once the message is received by the Odrive's subscriber it will send a CAN message to set the motor's velocity and velocity integrator gains and their limits. 

vel_gain - specifies the new velocity gain
vel_integrator_gain - specifies the new velocity integrator gain.
vel_integrator_limit - Limits the integrator output. Set to infinity to disable.
vel_limit - The limit on the velocity. Set to infinity to disable.

The topic of the ControlVelocityGains subscriber is set to "control_vel_gains". Though it should be noted that because of how the odrive_node works that in practice the topic will be the odrive's namespace followed by "control_vel_gains". E.g. "/pitch/odrv1/control_vel_gains".

## Control Position Gains ##

We've added a custom ROS2 message called ControlPositionGain to take care of setting the Position gain via messages. 

It's structure is shown below: 

float32 pos_gain

This is a message that will be sent to the Odrive.

Once the message is received by the Odrive's subscriber it will send a CAN message to set the motor's position gain.

pos_gain - specifies the new position gain

The topic of the ControlGains subscriber is set to "control_pos_gain". Though it should be noted that because of how the odrive_node works that in practice the topic will be the odrive's namespace followed by "control_pos_gain". E.g. "/pitch/odrv1/control_pos_gain".


### Control Traj Vel Limit ###

We've added a custom ROS2 message called ControlTrajVelLim to take care of setting the Traj_Vel_Limit via messages. 

Its structure is shown below: 

float32 traj_vel_limit

This is a message that will be sent to the Odrive.

Once the message is received by the Odrive's subscriber it will send a CAN message to set the traj_vel_limit property

traj_vel_limit - specifies the new limit for traj velocity

The topic of the ControlTrajVelLim subscriber is set to "control_traj_vel_lim". Though it should be noted that because of how the odrive_node works that in practice the topic will be the odrive's namespace followed by "control_vel_gains". E.g. "/pitch/odrv1/control_traj_vel_lim".

### Set Traj Accel Limits ###

We've added a custom ROS2 message called ControlTrajAccelLims to take care of setting the traj_accel_limit and the traj_decel_limit via messages. 

Its structure is shown below: 
  
float32 traj_accel_limit  
float32 traj_decel_limit  
  
This is a message that will be sent to the Odrive.

Once the message is received by the Odrive's subscriber it will send a CAN message to set the traj_accel_limit and the traj_decel_limit properties.

traj_accel_limit - specifies the new limit for traj acceleration
traj_decel_limit - specifies the new limit for traj deceleration

The topic of the ControlTrajVelLim subscriber is set to "control_traj_accel_lims". Though it should be noted that because of how the odrive_node works that in practice the topic will be the odrive's namespace followed by "control_traj_accel_lims". E.g. "/pitch/odrv1/control_traj_accel_lims".
  
## New Services ##

### ValueAccess ###

We've added a custom ROS service to access or change arbitrary values on the odrive. 

The structure of the service call is as follows:  

uint8 opcode   
uint16 endpoint_id  
uint32 data_type_specifier  
bool bool_value  
float32 float32_value  
int32 int32_value  
uint64 uint64_value  
uint32 uint32_value  
uint16 uint16_value  
uint8 uint8_value  
\-\-\-  
uint8 opcode  
uint16 endpoint_id  
bool bool_value  
float32 float32_value  
int32 int32_value  
uint64 uint64_value  
uint32 uint32_value  
uint16 uint16_value  
uint8 uint8_value  

This is a service that will be sent to the Odrive and the Odrive will send a response back to the client.

The first part of the service message is the structure of the request that we send through to the odrive. The second part of the service message is the structure of the response that we receive. 

The documentation for how to access or change arbirary values for the Odrive via CAN bus is detailed at this [Link](https://docs.odriverobotics.com/v/latest/guides/can-guide.html#can-endpoint-access). This service is a wrapper for the CAN functionality that works as follows:  

opcode - (0 - read, 1 - write) This is functionally a boolean we use to specify whether we want to read a particular value or write to it  

endpoint_id - This is the endpoint of the value as specified for the relevant version of the firmware. (It changes each firmware version so you should check [this list](https://docs.odriverobotics.com/releases/firmware) to find the right endpoint ids for your your version.

data_type_specifier - (0 - bool, 1 - float32, 2 - int32, 3 - uint64, 4 - uint32, 5 - uint16, 6 - uint8) This will specify the type of data that is being passed to the odrive or should be expected to be received. This is necessary otherwise our code won't know how to handle the values. 

The other parameters are just used to send or receive the values themselves. To work with ROS and C++ we had to make them separate parameters but only one should be populated at a time and it should be specified with the data_type_specifier.

The service name will be "access_value" combined with the odrive's namespace. E.g. "/pitch/odrv1/access_value"


### Estop ###

We've added a custom ROS service to estop the odrive using CAN bus. 

The structure of the service call is as follows:  
 
\-\-\-  
bool estopped_system

This is a service that will be sent to the Odrive and the Odrive will send a response back to the client.

The first part of the service message is the structure of the request that we send through to the odrive. The second part of the service message is the structure of the response that we receive. 

In this case the first part is blank because we don't send through any values for the request. 

estopped_system - this is a boolean that returns true if the system is estopped. At the moment there is no way to test that so it always returns True. (Service responses can't be blank so there has to be a value returned.

The service name will be "estop" combined with the odrive's namespace. E.g. "/pitch/odrv1/estop"

  
### Clear Errors ###


We've added a custom ROS service to clear the errors on the odrive using CAN bus. 

The structure of the service call is as follows:  
 
\-\-\-  
bool cleared_errors

This is a service that will be sent to the Odrive and the Odrive will send a response back to the client.

The first part of the service message is the structure of the request that we send through to the odrive. The second part of the service message is the structure of the response that we receive. 

In this case the first part is blank because we don't send through any values for the request. 

cleared_errors - this is a boolean that returns true if the odrive has cleared errors. At the moment there is no way to test that so it always returns True. (Service responses can't be blank so there has to be a value returned.

The service name will be "clear_errors" combined with the odrive's namespace. E.g. "/pitch/odrv1/clear_errors"

## New Parameters ##

### Config File ###

We've added the capability of setting arbitrary parameters on the odrive when the code first starts by getting values from a passed in .yaml file. 

This is controlled within the function ODriveCanNode::settingsFromConfig(). It pulls in parameters based on type specific dictionaries that are defined within it. For each data type of parameters that can be accessed (bool, float32, int32, uint64, uint32, uint16, uint8) there is a dictionary that contains key, value paris of parameter names matched with their endpoint ids. The endpoint ids you can find in the odrive documentation. To allow setting a new value from the config file you simply need to add the name of the parameter you are passing in and the endpoint where the parameter is found to the correct dictionary. You will need to update the odrive code itself in order to make this change. 
E.g. for the parameter config.odrv_fan.enabled which for the firmware version 0.6.9-1 is a boolean found at endpoint 192 you would add this line: 
bool_parameter_map["config.odrv_fan.enabled"] = 192;   

An example config yaml file is included with the code in the directory 'config'

## Other ##

### Comments ###

When adding custom code to this package we've put each block within comments tags that look like this.
  
// CUSTOM CODE START  
// CUSTOM CODE END  





## ODrive Original ReadMe Continued ##
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

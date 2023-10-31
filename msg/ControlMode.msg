uint32 mode

# Control Mode enumerations
# This mode is not used internally.
uint32 VOLTAGE_CONTROL = 0x0

# Uses only the inner torque control loop.
# Use input_torque to command desired torque. 
uint32 TORQUE_CONTROL = 0x1

# Uses both the inner torque control loop and the velocity control loop.
# Use input_vel to command desired velocity, and input_torque.
uint32 VELOCITY_CONTROL = 0x2

# Uses the inner torque loop, the velocity control loop, and the outer position control loop.
# Use input_pos to command desired position, input_vel to command velocity feed-forward, and input_torque for torque feed-forward.
uint32 POSITION_CONTROL = 0x3

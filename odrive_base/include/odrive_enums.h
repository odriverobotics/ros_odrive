
#ifndef ODriveEnums_h
#define ODriveEnums_h

// ODrive.GpioMode
enum ODriveGpioMode {
    GPIO_MODE_DIGITAL                        = 0,
    GPIO_MODE_DIGITAL_PULL_UP                = 1,
    GPIO_MODE_DIGITAL_PULL_DOWN              = 2,
    GPIO_MODE_ANALOG_IN                      = 3,
    GPIO_MODE_UART_A                         = 4,
    GPIO_MODE_UART_B                         = 5,
    GPIO_MODE_UART_C                         = 6,
    GPIO_MODE_CAN_A                          = 7,
    GPIO_MODE_I2C_A                          = 8,
    GPIO_MODE_SPI_A                          = 9,
    GPIO_MODE_PWM                            = 10,
    GPIO_MODE_ENC0                           = 11,
    GPIO_MODE_ENC1                           = 12,
    GPIO_MODE_ENC2                           = 13,
    GPIO_MODE_MECH_BRAKE                     = 14,
    GPIO_MODE_STATUS                         = 15,
    GPIO_MODE_BRAKE_RES                      = 16,
    GPIO_MODE_AUTO                           = 17,
};

// ODrive.StreamProtocolType
enum ODriveStreamProtocolType {
    STREAM_PROTOCOL_TYPE_FIBRE               = 0,
    STREAM_PROTOCOL_TYPE_ASCII               = 1,
    STREAM_PROTOCOL_TYPE_STDOUT              = 2,
    STREAM_PROTOCOL_TYPE_ASCII_AND_STDOUT    = 3,
    STREAM_PROTOCOL_TYPE_OTHER               = 4,
};

// ODrive.Can.Protocol
enum ODriveProtocol {
    PROTOCOL_NONE                            = 0x00000000,
    PROTOCOL_SIMPLE                          = 0x00000001,
};

// ODrive.Axis.AxisState
enum ODriveAxisState {
    AXIS_STATE_UNDEFINED                     = 0,
    AXIS_STATE_IDLE                          = 1,
    AXIS_STATE_STARTUP_SEQUENCE              = 2,
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE     = 3,
    AXIS_STATE_MOTOR_CALIBRATION             = 4,
    AXIS_STATE_ENCODER_INDEX_SEARCH          = 6,
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION    = 7,
    AXIS_STATE_CLOSED_LOOP_CONTROL           = 8,
    AXIS_STATE_LOCKIN_SPIN                   = 9,
    AXIS_STATE_ENCODER_DIR_FIND              = 10,
    AXIS_STATE_HOMING                        = 11,
    AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION = 12,
    AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION = 13,
    AXIS_STATE_ANTICOGGING_CALIBRATION       = 14,
};

// ODrive.Controller.ControlMode
enum ODriveControlMode {
    CONTROL_MODE_VOLTAGE_CONTROL             = 0,
    CONTROL_MODE_TORQUE_CONTROL              = 1,
    CONTROL_MODE_VELOCITY_CONTROL            = 2,
    CONTROL_MODE_POSITION_CONTROL            = 3,
};

// ODrive.ComponentStatus
enum ODriveComponentStatus {
    COMPONENT_STATUS_NOMINAL                 = 0,
    COMPONENT_STATUS_NO_RESPONSE             = 1,
    COMPONENT_STATUS_INVALID_RESPONSE_LENGTH = 2,
    COMPONENT_STATUS_PARITY_MISMATCH         = 3,
    COMPONENT_STATUS_ILLEGAL_HALL_STATE      = 4,
    COMPONENT_STATUS_POLARITY_NOT_CALIBRATED = 5,
    COMPONENT_STATUS_PHASES_NOT_CALIBRATED   = 6,
    COMPONENT_STATUS_NUMERICAL_ERROR         = 7,
    COMPONENT_STATUS_MISSING_INPUT           = 8,
    COMPONENT_STATUS_RELATIVE_MODE           = 9,
    COMPONENT_STATUS_UNCONFIGURED            = 10,
    COMPONENT_STATUS_OVERSPEED               = 11,
    COMPONENT_STATUS_INDEX_NOT_FOUND         = 12,
    COMPONENT_STATUS_BAD_CONFIG              = 13,
    COMPONENT_STATUS_NOT_ENABLED             = 14,
    COMPONENT_STATUS_SPINOUT_DETECTED        = 15,
};

// ODrive.Error
enum ODriveError {
    ODRIVE_ERROR_NONE                        = 0x00000000,
    ODRIVE_ERROR_INITIALIZING                = 0x00000001,
    ODRIVE_ERROR_SYSTEM_LEVEL                = 0x00000002,
    ODRIVE_ERROR_TIMING_ERROR                = 0x00000004,
    ODRIVE_ERROR_MISSING_ESTIMATE            = 0x00000008,
    ODRIVE_ERROR_BAD_CONFIG                  = 0x00000010,
    ODRIVE_ERROR_DRV_FAULT                   = 0x00000020,
    ODRIVE_ERROR_MISSING_INPUT               = 0x00000040,
    ODRIVE_ERROR_DC_BUS_OVER_VOLTAGE         = 0x00000100,
    ODRIVE_ERROR_DC_BUS_UNDER_VOLTAGE        = 0x00000200,
    ODRIVE_ERROR_DC_BUS_OVER_CURRENT         = 0x00000400,
    ODRIVE_ERROR_DC_BUS_OVER_REGEN_CURRENT   = 0x00000800,
    ODRIVE_ERROR_CURRENT_LIMIT_VIOLATION     = 0x00001000,
    ODRIVE_ERROR_MOTOR_OVER_TEMP             = 0x00002000,
    ODRIVE_ERROR_INVERTER_OVER_TEMP          = 0x00004000,
    ODRIVE_ERROR_VELOCITY_LIMIT_VIOLATION    = 0x00008000,
    ODRIVE_ERROR_POSITION_LIMIT_VIOLATION    = 0x00010000,
    ODRIVE_ERROR_WATCHDOG_TIMER_EXPIRED      = 0x01000000,
    ODRIVE_ERROR_ESTOP_REQUESTED             = 0x02000000,
    ODRIVE_ERROR_SPINOUT_DETECTED            = 0x04000000,
    ODRIVE_ERROR_BRAKE_RESISTOR_DISARMED     = 0x08000000,
    ODRIVE_ERROR_THERMISTOR_DISCONNECTED     = 0x10000000,
    ODRIVE_ERROR_CALIBRATION_ERROR           = 0x40000000,
};

// ODrive.ProcedureResult
enum ODriveProcedureResult {
    PROCEDURE_RESULT_SUCCESS                 = 0,
    PROCEDURE_RESULT_BUSY                    = 1,
    PROCEDURE_RESULT_CANCELLED               = 2,
    PROCEDURE_RESULT_DISARMED                = 3,
    PROCEDURE_RESULT_NO_RESPONSE             = 4,
    PROCEDURE_RESULT_POLE_PAIR_CPR_MISMATCH  = 5,
    PROCEDURE_RESULT_PHASE_RESISTANCE_OUT_OF_RANGE = 6,
    PROCEDURE_RESULT_PHASE_INDUCTANCE_OUT_OF_RANGE = 7,
    PROCEDURE_RESULT_UNBALANCED_PHASES       = 8,
    PROCEDURE_RESULT_INVALID_MOTOR_TYPE      = 9,
    PROCEDURE_RESULT_ILLEGAL_HALL_STATE      = 10,
    PROCEDURE_RESULT_TIMEOUT                 = 11,
    PROCEDURE_RESULT_HOMING_WITHOUT_ENDSTOP  = 12,
    PROCEDURE_RESULT_INVALID_STATE           = 13,
    PROCEDURE_RESULT_NOT_CALIBRATED          = 14,
    PROCEDURE_RESULT_NOT_CONVERGING          = 15,
};

// ODrive.EncoderId
enum ODriveEncoderId {
    ENCODER_ID_NONE                          = 0,
    ENCODER_ID_INC_ENCODER0                  = 1,
    ENCODER_ID_INC_ENCODER1                  = 2,
    ENCODER_ID_INC_ENCODER2                  = 3,
    ENCODER_ID_SENSORLESS_ESTIMATOR          = 4,
    ENCODER_ID_SPI_ENCODER0                  = 5,
    ENCODER_ID_SPI_ENCODER1                  = 6,
    ENCODER_ID_SPI_ENCODER2                  = 7,
    ENCODER_ID_HALL_ENCODER0                 = 8,
    ENCODER_ID_HALL_ENCODER1                 = 9,
    ENCODER_ID_RS485_ENCODER0                = 10,
    ENCODER_ID_RS485_ENCODER1                = 11,
    ENCODER_ID_RS485_ENCODER2                = 12,
    ENCODER_ID_ONBOARD_ENCODER0              = 13,
    ENCODER_ID_ONBOARD_ENCODER1              = 14,
};

// ODrive.SpiEncoderMode
enum ODriveSpiEncoderMode {
    SPI_ENCODER_MODE_DISABLED                = 0,
    SPI_ENCODER_MODE_RLS                     = 1,
    SPI_ENCODER_MODE_AMS                     = 2,
    SPI_ENCODER_MODE_CUI                     = 3,
    SPI_ENCODER_MODE_AEAT                    = 4,
    SPI_ENCODER_MODE_MA732                   = 5,
    SPI_ENCODER_MODE_TLE                     = 6,
    SPI_ENCODER_MODE_BISSC                   = 7,
};

// ODrive.Rs485EncoderMode
enum ODriveRs485EncoderMode {
    RS485_ENCODER_MODE_DISABLED              = 0,
    RS485_ENCODER_MODE_AMT21_POLLING         = 1,
    RS485_ENCODER_MODE_AMT21_EVENT_DRIVEN    = 2,
    RS485_ENCODER_MODE_MBS                   = 3,
    RS485_ENCODER_MODE_ODRIVE_OA1            = 4,
};

// ODrive.Controller.InputMode
enum ODriveInputMode {
    INPUT_MODE_INACTIVE                      = 0,
    INPUT_MODE_PASSTHROUGH                   = 1,
    INPUT_MODE_VEL_RAMP                      = 2,
    INPUT_MODE_POS_FILTER                    = 3,
    INPUT_MODE_MIX_CHANNELS                  = 4,
    INPUT_MODE_TRAP_TRAJ                     = 5,
    INPUT_MODE_TORQUE_RAMP                   = 6,
    INPUT_MODE_MIRROR                        = 7,
    INPUT_MODE_TUNING                        = 8,
};

// ODrive.MotorType
enum ODriveMotorType {
    MOTOR_TYPE_HIGH_CURRENT                  = 0,
    MOTOR_TYPE_GIMBAL                        = 2,
    MOTOR_TYPE_ACIM                          = 3,
};

// ODrive.Can.Error
enum ODriveCanError {
    CAN_ERROR_NONE                           = 0x00000000,
    CAN_ERROR_DUPLICATE_CAN_IDS              = 0x00000001,
    CAN_ERROR_BUS_OFF                        = 0x00000002,
    CAN_ERROR_LOW_LEVEL                      = 0x00000004,
    CAN_ERROR_PROTOCOL_INIT                  = 0x00000008,
};

#endif
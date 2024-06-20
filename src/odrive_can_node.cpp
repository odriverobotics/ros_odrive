#include "odrive_can_node.hpp"
#include "epoll_event_loop.hpp"
#include "byte_swap.hpp"
#include <sys/eventfd.h>
#include <chrono>

enum CmdId : uint32_t {
    // CUSTOM CODE START
    kGet_Version = 0x000,
    // CUSTOM CODE END

    kHeartbeat = 0x001,            // ControllerStatus  - publisher

    // CUSTOM CODE START
    kEstop = 0x002,
    // CUSTOM CODE END

    kGetError = 0x003,             // SystemStatus      - publisher

    // CUSTOM CODE START
    kRxSdo = 0x004,
    kTxSdo = 0x005,
    kAddress = 0x006,
    // CUSTOM CODE END

    kSetAxisState = 0x007,         // SetAxisState      - service
    kGetEncoderEstimates = 0x009,  // ControllerStatus  - publisher
    kSetControllerMode = 0x00b,    // ControlMessage    - subscriber
    kSetInputPos = 0x00c,                  // ControlMessage    - subscriber
    kSetInputVel = 0x00d,                  // ControlMessage    - subscriber
    kSetInputTorque = 0x00e,               // ControlMessage    - subscriber
    // CUSTOM CODE START
    kSetLimits = 0x00f,
    kSetTrajVelLimit = 0x011,
    kSetTrajAccelLimit = 0x012,
    kSetTragInertia = 0x013,
    // CUSTOM CODE END
    kGetIq = 0x014,                // ControllerStatus  - publisher
    kGetTemp = 0x015,                      // SystemStatus      - publisher
    
    // CUSTOM CODE START
    kReboot = 0x016,

    // CUSTOM CODE END

    kGetBusVoltageCurrent = 0x017, // SystemStatus      - publisher



    // CUSTOM CODE START

    kClearErrors = 0x018,
    kSetAbsolutePostion = 0x019,
    kSetPosGain = 0x01a,    
   
    kSetVelGains = 0x01b,           //ControlGains - Subscriber
    // CUSTOM CODE END

    kGetTorques = 0x01c,           // ControllerStatus  - publisher

    // CUSTOM CODE START
    kGetPowers = 0x01d,
    kEnterDFUMode = 0x01f,
    // CUSTOM CODE END
};

enum ControlMode : uint64_t {
    kVoltageControl,
    kTorqueControl,
    kVelocityControl,
    kPositionControl,
};

ODriveCanNode::ODriveCanNode(const std::string& node_name) : rclcpp::Node(node_name) {
    
    rclcpp::Node::declare_parameter<std::string>("interface", "can0");
    rclcpp::Node::declare_parameter<uint16_t>("node_id", 0);

    rclcpp::QoS ctrl_stat_qos(rclcpp::KeepAll{});
    ctrl_publisher_ = rclcpp::Node::create_publisher<ControllerStatus>("controller_status", ctrl_stat_qos);
    
    rclcpp::QoS odrv_stat_qos(rclcpp::KeepAll{});
    odrv_publisher_ = rclcpp::Node::create_publisher<ODriveStatus>("odrive_status", odrv_stat_qos);

    rclcpp::QoS ctrl_msg_qos(rclcpp::KeepAll{});
    subscriber_ = rclcpp::Node::create_subscription<ControlMessage>("control_message", ctrl_msg_qos, std::bind(&ODriveCanNode::subscriber_callback, this, _1));

    rclcpp::QoS srv_qos(rclcpp::KeepAll{});
    service_ = rclcpp::Node::create_service<AxisState>("request_axis_state", std::bind(&ODriveCanNode::service_callback, this, _1, _2), srv_qos.get_rmw_qos_profile());

    // CUSTOM CODE START


    rclcpp::QoS odrv_advanced_stat_qos(rclcpp::KeepLast(10));
    odrv_advanced_publisher_ = rclcpp::Node::create_publisher<ODriveStatusAdvanced>("odrive_status_advanced", odrv_advanced_stat_qos);

    rclcpp::QoS gains_subscriber_qos(rclcpp::KeepAll{});
    gains_subscriber_ = rclcpp::Node::create_subscription<ControlGains>("control_gains", gains_subscriber_qos, std::bind(&ODriveCanNode::control_gains_callback, this, _1));

   
    rclcpp::QoS value_access_srv_qos(rclcpp::KeepAll{});
    value_access_service_ = rclcpp::Node::create_service<ValueAccess>("access_value", std::bind(&ODriveCanNode::value_access_service_callback, this, _1, _2), value_access_srv_qos.get_rmw_qos_profile());



    // CUSTOM CODE END

}

void ODriveCanNode::deinit() {
    sub_evt_.deinit();
    srv_evt_.deinit();
    can_intf_.deinit();
}

bool ODriveCanNode::init(EpollEventLoop* event_loop) {

    node_id_ = rclcpp::Node::get_parameter("node_id").as_int();
    std::string interface = rclcpp::Node::get_parameter("interface").as_string();

    if (!can_intf_.init(interface, event_loop, std::bind(&ODriveCanNode::recv_callback, this, _1))) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to initialize socket can interface: %s", interface.c_str());
        return false;
    }
    if (!sub_evt_.init(event_loop, std::bind(&ODriveCanNode::ctrl_msg_callback, this))) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to initialize subscriber event");
        return false;
    }
    if (!srv_evt_.init(event_loop, std::bind(&ODriveCanNode::request_state_callback, this))) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to initialize service event");
        return false;
    }
    RCLCPP_INFO(rclcpp::Node::get_logger(), "node_id: %d", node_id_);
    RCLCPP_INFO(rclcpp::Node::get_logger(), "interface: %s", interface.c_str());
    return true;
}

void ODriveCanNode::recv_callback(const can_frame& frame) {

    if(((frame.can_id >> 5) & 0x3F) != node_id_) return;

    // checks the lower 5 bits of the can_id which store the cmd_id
    // ost → ODrive

    switch(frame.can_id & 0x1F) {
        case CmdId::kHeartbeat: {
            if (!verify_length("kHeartbeat", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
            ctrl_stat_.active_errors    = read_le<uint32_t>(frame.data + 0);
            ctrl_stat_.axis_state        = read_le<uint8_t>(frame.data + 4);
            ctrl_stat_.procedure_result  = read_le<uint8_t>(frame.data + 5);
            ctrl_stat_.trajectory_done_flag = read_le<bool>(frame.data + 6);
            ctrl_pub_flag_ |= 0b0001;
            fresh_heartbeat_.notify_one();

            // CUSTOM CODE START
            std::lock_guard<std::mutex> guard1(odrv_advanced_stat_mutex_);
            odrv_advanced_stat_.ctrl_active_errors    = read_le<uint32_t>(frame.data + 0);
            odrv_advanced_stat_.axis_state        = read_le<uint8_t>(frame.data + 4);
            odrv_advanced_stat_.procedure_result  = read_le<uint8_t>(frame.data + 5);
            odrv_advanced_stat_.trajectory_done_flag = read_le<bool>(frame.data + 6);
            odrv_advanced_ctrl_pub_flag_ |= 0b0001;
            // CUSTOM CODE END

            break;
        }
        case CmdId::kGetError: {
            if (!verify_length("kGetError", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(odrv_stat_mutex_);
            odrv_stat_.active_errors = read_le<uint32_t>(frame.data + 0);
            odrv_stat_.disarm_reason = read_le<uint32_t>(frame.data + 4);
            odrv_pub_flag_ |= 0b001;

            // CUSTOM CODE START
            std::lock_guard<std::mutex> guard1(odrv_advanced_stat_mutex_);
            odrv_advanced_stat_.active_errors = odrv_stat_.active_errors;
            odrv_advanced_stat_.disarm_reason = odrv_stat_.disarm_reason;
            odrv_advanced_pub_flag_ |= 0b001;
            // CUSTOM CODE END


            break;
        }
        case CmdId::kGetEncoderEstimates: {
            if (!verify_length("kGetEncoderEstimates", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
            ctrl_stat_.pos_estimate = read_le<float>(frame.data + 0);
            ctrl_stat_.vel_estimate = read_le<float>(frame.data + 4);
            ctrl_pub_flag_ |= 0b0010;

            // CUSTOM CODE START
            std::lock_guard<std::mutex> guard1(odrv_advanced_stat_mutex_);
            odrv_advanced_stat_.pos_estimate = read_le<float>(frame.data + 0);
            odrv_advanced_stat_.vel_estimate = read_le<float>(frame.data + 4);
            odrv_advanced_ctrl_pub_flag_ |= 0b0010;
            // CUSTOM CODE END
            break;
        }
        case CmdId::kGetIq: {
            if (!verify_length("kGetIq", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
            ctrl_stat_.iq_setpoint = read_le<float>(frame.data + 0);
            ctrl_stat_.iq_measured = read_le<float>(frame.data + 4);
            ctrl_pub_flag_ |= 0b0100;

            // CUSTOM CODE START
            std::lock_guard<std::mutex> guard1(odrv_advanced_stat_mutex_);
            odrv_advanced_stat_.iq_setpoint = read_le<float>(frame.data + 0);
            odrv_advanced_stat_.iq_measured = read_le<float>(frame.data + 4);
            odrv_advanced_ctrl_pub_flag_ |= 0b0100;
            // CUSTOM CODE END
            break;
        }
        case CmdId::kGetTemp: {
            if (!verify_length("kGetTemp", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(odrv_stat_mutex_);
            odrv_stat_.fet_temperature   = read_le<float>(frame.data + 0);
            odrv_stat_.motor_temperature = read_le<float>(frame.data + 4);
            odrv_pub_flag_ |= 0b010;

            // CUSTOM CODE START
            std::lock_guard<std::mutex> guard1(odrv_advanced_stat_mutex_);
            odrv_advanced_stat_.fet_temperature = odrv_stat_.fet_temperature;
            odrv_advanced_stat_.motor_temperature = odrv_stat_.motor_temperature;
            odrv_advanced_pub_flag_ |= 0b010;
            // CUSTOM CODE END
            break;
        }
        case CmdId::kGetBusVoltageCurrent: {
            if (!verify_length("kGetBusVoltageCurrent", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(odrv_stat_mutex_);
            odrv_stat_.bus_voltage = read_le<float>(frame.data + 0);
            odrv_stat_.bus_current = read_le<float>(frame.data + 4);
            odrv_pub_flag_ |= 0b100;

            // CUSTOM CODE START
            std::lock_guard<std::mutex> guard1(odrv_advanced_stat_mutex_);
            odrv_advanced_stat_.bus_voltage = odrv_stat_.bus_voltage;
            odrv_advanced_stat_.bus_current = odrv_stat_.bus_current;
            odrv_advanced_pub_flag_ |= 0b100;
            // CUSTOM CODE END
            break;
        }
        case CmdId::kGetTorques: {
            if (!verify_length("kGetTorques", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
            ctrl_stat_.torque_target   = read_le<float>(frame.data + 0);
            ctrl_stat_.torque_estimate = read_le<float>(frame.data + 4);
            ctrl_pub_flag_ |= 0b1000; 

            // CUSTOM CODE START
            std::lock_guard<std::mutex> guard1(odrv_advanced_stat_mutex_);
            odrv_advanced_stat_.torque_target   = read_le<float>(frame.data + 0);
            odrv_advanced_stat_.torque_estimate = read_le<float>(frame.data + 4);
            
            odrv_advanced_ctrl_pub_flag_ |= 0b1000;
            // CUSTOM CODE END

            break;
        }

        // CUSTOM CODE START

        case CmdId::kTxSdo: {
            if (!verify_length("kTxSdo", 8, frame.can_dlc)) break;
            {
            std::lock_guard<std::mutex> guard(value_access_mutex_);
            


            // reads in the correct datatype
            switch (value_access_datatype_specifier_) {

                case 0: {
                    // bool
                    RCLCPP_DEBUG(rclcpp::Node::get_logger(), "read value type was bool");
                    value_access_reponse_.bool_value   = read_le<bool>(frame.data + 4);
                    break;
                    
                }
                case 1: {
                    // float32
                    RCLCPP_DEBUG(rclcpp::Node::get_logger(), "read value type was float32");
                    value_access_reponse_.float32_value   = read_le<float>(frame.data + 4);
                    break;
                   
                }
                case 2: {
                    // int32
                    RCLCPP_DEBUG(rclcpp::Node::get_logger(), "read value type was int32");
                    value_access_reponse_.int32_value   = read_le<int32_t>(frame.data + 4);
                    break;

                }
                case 3: {
                    // uint64
                    RCLCPP_DEBUG(rclcpp::Node::get_logger(), "read value type was uint64");
                    value_access_reponse_.uint64_value   = read_le<uint64_t>(frame.data + 4);
                    break;

                }    
                case 4: {
                    // uint32
                    RCLCPP_DEBUG(rclcpp::Node::get_logger(), "read value type was uint32");
                    value_access_reponse_.uint32_value   = read_le<uint32_t>(frame.data + 4);
                    break;

                }
                case 5: {
                    // uint16
                    RCLCPP_DEBUG(rclcpp::Node::get_logger(), "read value type was uint16");
                    value_access_reponse_.uint16_value   = read_le<uint16_t>(frame.data + 4);
                    break;

                }
                case 6: {
                    // uint8
                    RCLCPP_DEBUG(rclcpp::Node::get_logger(), "read value type was uint8");
                    value_access_reponse_.uint8_value   = read_le<uint8_t>(frame.data + 4);
                    break;

                }
                default: 
                    RCLCPP_ERROR(rclcpp::Node::get_logger(), "unsupported data type specified: %d", value_access_datatype_specifier_);
                    return;
            }   

            value_access_datatype_specifier_ = 0;
            received_TxSdo_ = true;
            fresh_TxSdo_.notify_one();

            }

            
            break;
        }

        // CUSTOM CODE END
        default: {
            RCLCPP_WARN(rclcpp::Node::get_logger(), "Received unused message: ID = 0x%x", (frame.can_id & 0x1F));
            break;
        }
    }
    
    if (ctrl_pub_flag_ == 0b1111) {
        ctrl_publisher_->publish(ctrl_stat_);
        ctrl_pub_flag_ = 0;
    }
    
    if (odrv_pub_flag_ == 0b111) {
        odrv_publisher_->publish(odrv_stat_);
        odrv_pub_flag_ = 0;
    }

    // CUSTOM CODE START
    if (odrv_advanced_pub_flag_ == 0b111) {

        if(odrv_advanced_ctrl_pub_flag_ == 0b1111){
            odrv_advanced_publisher_->publish(odrv_advanced_stat_);
            odrv_advanced_pub_flag_ = 0;
            odrv_advanced_ctrl_pub_flag_ = 0;
        }
        
        
    }

    // CUSTOM CODE END
}

void ODriveCanNode::subscriber_callback(const ControlMessage::SharedPtr msg) {
    std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
    ctrl_msg_ = *msg;
    sub_evt_.set();
}

void ODriveCanNode::service_callback(const std::shared_ptr<AxisState::Request> request, std::shared_ptr<AxisState::Response> response) {
    {
        std::unique_lock<std::mutex> guard(axis_state_mutex_);
        axis_state_ = request->axis_requested_state;
        RCLCPP_INFO(rclcpp::Node::get_logger(), "requesting axis state: %d", axis_state_);
    }
    srv_evt_.set();

    std::unique_lock<std::mutex> guard(ctrl_stat_mutex_); // define lock for controller status
    auto call_time = std::chrono::steady_clock::now();
    fresh_heartbeat_.wait(guard, [this, &call_time]() {
        bool complete = (this->ctrl_stat_.procedure_result != 1) && // make sure procedure_result is not busy
            (std::chrono::steady_clock::now() - call_time >= std::chrono::seconds(1)); // wait for minimum one second 
        return complete; 
        }); // wait for procedure_result
    
    response->axis_state = ctrl_stat_.axis_state;
    response->active_errors = ctrl_stat_.active_errors;
    response->procedure_result = ctrl_stat_.procedure_result;
}

void ODriveCanNode::request_state_callback() {
    struct can_frame frame;
    frame.can_id = node_id_ << 5 | CmdId::kSetAxisState;
    {
        std::unique_lock<std::mutex> guard(axis_state_mutex_);
        write_le<uint32_t>(axis_state_, frame.data);
    }
    frame.can_dlc = 4;
    can_intf_.send_can_frame(frame);
}

void ODriveCanNode::ctrl_msg_callback() {

    uint32_t control_mode;
    struct can_frame frame;
    frame.can_id = node_id_ << 5 | kSetControllerMode;
    {
        std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
        write_le<uint32_t>(ctrl_msg_.control_mode, frame.data);
        write_le<uint32_t>(ctrl_msg_.input_mode,   frame.data + 4);
        control_mode = ctrl_msg_.control_mode;
    }
    frame.can_dlc = 8;
    can_intf_.send_can_frame(frame);
    
    frame = can_frame{};
    switch (control_mode) {
        case ControlMode::kVoltageControl: {
            RCLCPP_ERROR(rclcpp::Node::get_logger(), "Voltage Control Mode (0) is not currently supported");
            return;
        }
        case ControlMode::kTorqueControl: {
            RCLCPP_DEBUG(rclcpp::Node::get_logger(), "input_torque");
            frame.can_id = node_id_ << 5 | kSetInputTorque;
            std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
            write_le<float>(ctrl_msg_.input_torque, frame.data);
            frame.can_dlc = 4;
            break;
        }
        case ControlMode::kVelocityControl: {
            RCLCPP_DEBUG(rclcpp::Node::get_logger(), "input_vel");
            frame.can_id = node_id_ << 5 | kSetInputVel;
            std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
            write_le<float>(ctrl_msg_.input_vel,       frame.data);
            write_le<float>(ctrl_msg_.input_torque, frame.data + 4);
            frame.can_dlc = 8;
            break;
        }
        case ControlMode::kPositionControl: {
            RCLCPP_DEBUG(rclcpp::Node::get_logger(), "input_pos");
            frame.can_id = node_id_ << 5 | kSetInputPos;
            std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
            write_le<float>(ctrl_msg_.input_pos,  frame.data);
            write_le<int8_t>(((int8_t)((ctrl_msg_.input_vel) * 1000)),    frame.data + 4);
            write_le<int8_t>(((int8_t)((ctrl_msg_.input_torque) * 1000)), frame.data + 6);
            frame.can_dlc = 8;
            break;
        }    
        default: 
            RCLCPP_ERROR(rclcpp::Node::get_logger(), "unsupported control_mode: %d", control_mode);
            return;
    }

    can_intf_.send_can_frame(frame);
}

// CUSTOM CODE START
// Trying to send an advanced control message


void ODriveCanNode::control_gains_callback(const odrive_can::msg::ControlGains::SharedPtr msg) {

    RCLCPP_INFO(rclcpp::Node::get_logger(), "GAINS FEEDBACK!!!");
    RCLCPP_INFO(rclcpp::Node::get_logger(), "vel gain: %f", msg->vel_gain);
    RCLCPP_INFO(rclcpp::Node::get_logger(), "vel integrator gain: %f", msg->vel_integrator_gain);


    struct can_frame frame;
    frame.can_id = node_id_ << 5 | kSetVelGains;
    {
        std::lock_guard<std::mutex> guard(gains_msg_mutex_);
        write_le<float>(msg->vel_gain, frame.data);
        write_le<float>(msg->vel_integrator_gain,   frame.data + 4);
    }
    frame.can_dlc = 8;
    can_intf_.send_can_frame(frame);

    RCLCPP_INFO(rclcpp::Node::get_logger(), "GAINS UPDATED!!!");
    
    
}


// CUSTOM CODE END









void ODriveCanNode::value_access_service_callback(const std::shared_ptr<ValueAccess::Request> request, std::shared_ptr<ValueAccess::Response> response) {
    
    
    {
        std::unique_lock<std::mutex> guard(value_access_mutex_);
        value_access_datatype_specifier_ = request->data_type_specifier;
        

        // RCLCPP_INFO(rclcpp::Node::get_logger(), "SETTING VALUE or GETTTING VALUE");
        // RCLCPP_INFO(rclcpp::Node::get_logger(), "opcode: %d", request->opcode);
        // RCLCPP_INFO(rclcpp::Node::get_logger(), "datatype specifier: %d", request->data_type_specifier);

        received_TxSdo_ = false;
        value_access_reponse_.opcode = request->opcode;
        value_access_reponse_.endpoint_id = request->endpoint_id;
        


        struct can_frame frame;
        frame.can_id = node_id_ << 5 | kRxSdo;
        {
            write_le<uint8_t>(request->opcode, frame.data);
            write_le<uint16_t>(request->endpoint_id, frame.data + 1);
            if(request->opcode == 0){
                // If the message is asking to read a value then don't write the value to CAN

            }else if(request->opcode == 1) {
                // If the message is asking to write a value then handle that


                switch (value_access_datatype_specifier_) {

                    case 0: {
                        // bool
                        RCLCPP_DEBUG(rclcpp::Node::get_logger(), "value type was bool");
                        write_le<bool>(request->bool_value,   frame.data + 4);
                        break;

                    }
                    case 1: {
                        // float32
                        RCLCPP_DEBUG(rclcpp::Node::get_logger(), "value type was float32");
                        write_le<float>(request->float32_value,   frame.data + 4);
                        break;

                    }
                    case 2: {
                        // int32
                        RCLCPP_DEBUG(rclcpp::Node::get_logger(), "value type was int32");
                        write_le<int32_t>(request->int32_value,   frame.data + 4);
                        break;

                    }
                    case 3: {
                        // uint64
                        RCLCPP_DEBUG(rclcpp::Node::get_logger(), "value type was uint64");
                        write_le<uint64_t>(request->uint64_value,   frame.data + 4);
                        break;

                    }    
                    case 4: {
                        // uint32
                        RCLCPP_DEBUG(rclcpp::Node::get_logger(), "value type was uint32");
                        write_le<uint32_t>(request->uint32_value,   frame.data + 4);
                        break;

                    }
                    case 5: {
                        // uint16
                        RCLCPP_DEBUG(rclcpp::Node::get_logger(), "value type was uint16");
                        write_le<uint16_t>(request->uint16_value,   frame.data + 4);
                        break;

                    }
                    case 6: {
                        // uint8
                        RCLCPP_DEBUG(rclcpp::Node::get_logger(), "value type was uint8");
                        write_le<uint8_t>(request->uint8_value,   frame.data + 4);
                        break;

                    }
                    default: 
                        RCLCPP_ERROR(rclcpp::Node::get_logger(), "unsupported data type specified: %d", request->data_type_specifier);
                        return;
                }   

            }
            

        }
        frame.can_dlc = 8;
        can_intf_.send_can_frame(frame);


        if(request->opcode == 1){
            // If the opcode was 1 then we also send out a read request to get back the value that was set
            struct can_frame read_frame;
            read_frame.can_id = node_id_ << 5 | kRxSdo;
        
            write_le<uint8_t>(0, read_frame.data);
            write_le<uint16_t>(request->endpoint_id, read_frame.data + 1);

            read_frame.can_dlc = 8;
            can_intf_.send_can_frame(read_frame);
        }
        // 

        RCLCPP_INFO(rclcpp::Node::get_logger(), "END OF SETTING VALUE Request!!!");

        }

        std::unique_lock<std::mutex> guard(value_access_mutex_); //
        auto call_time = std::chrono::steady_clock::now();
        fresh_TxSdo_.wait(guard, [this, &call_time]() {
            return received_TxSdo_; 
            }); // wait for procedure_result

        
        *response = value_access_reponse_;
        value_access_reponse_ = ValueAccess::Response();
        RCLCPP_INFO(rclcpp::Node::get_logger(), "Got Service response!!!");
}


// CUSTOM CODE END

inline bool ODriveCanNode::verify_length(const std::string&name, uint8_t expected, uint8_t length) {
    bool valid = expected == length;
    RCLCPP_DEBUG(rclcpp::Node::get_logger(), "received %s", name.c_str());
    if (!valid) RCLCPP_WARN(rclcpp::Node::get_logger(), "Incorrect %s frame length: %d != %d", name.c_str(), length, expected);
    return valid;
}

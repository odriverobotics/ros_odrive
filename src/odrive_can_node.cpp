#include "odrive_can_node.hpp"
#include "epoll_event_loop.hpp"
#include "byte_swap.hpp"
#include <sys/eventfd.h>
#include <chrono>

enum CmdId : uint32_t {
    // This enum structure is where we assign a name to each cmd_id for the CAN protocols
    // This was limited to a few protocols in the original ordrive code but this is now the complete list


    // CUSTOM CODE START
    kGet_Version = 0x000,
    // CUSTOM CODE END

    kHeartbeat = 0x001,            // ControllerStatus  - publisher

    // CUSTOM CODE START
    kEstop = 0x002,
    // CUSTOM CODE END

    kGetError = 0x003,             // SystemStatus      - publisher

    // CUSTOM CODE START
    kRxSdo = 0x004, // The protocol to read/write to arbitrary parameters
    kTxSdo = 0x005, // The protocol for when the Odrive receives a CAN response to a read request sent with RxSdo
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
    kSetTrajInertia = 0x013,
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
   
    kSetVelGains = 0x01b,           //ControlVelocityGains - Subscriber (The protocol for setting the velocity and velocity integrator gains)
    // CUSTOM CODE END

    kGetTorques = 0x01c,           // ControllerStatus  - publisher

    // CUSTOM CODE START
    kGetPowers = 0x01d,
    kEnterDFUMode = 0x01f,
    // CUSTOM CODE END
};

enum ControlMode : uint64_t {
    // This enum assigns names to the control mode identifiers so it is easier to parse when the msg is recieved from the CAN Bus
    kVoltageControl,
    kTorqueControl,
    kVelocityControl,
    kPositionControl,
};

ODriveCanNode::ODriveCanNode(const std::string& node_name) : rclcpp::Node(node_name) {
    
    rclcpp::Node::declare_parameter<std::string>("interface", "can0");
    rclcpp::Node::declare_parameter<uint16_t>("node_id", 0);

    rclcpp::QoS ctrl_stat_qos(rclcpp::KeepLast(10));
    ctrl_publisher_ = rclcpp::Node::create_publisher<ControllerStatus>("controller_status", ctrl_stat_qos);
    
    rclcpp::QoS odrv_stat_qos(rclcpp::KeepLast(10));
    odrv_publisher_ = rclcpp::Node::create_publisher<ODriveStatus>("odrive_status", odrv_stat_qos);

    rclcpp::QoS ctrl_msg_qos(rclcpp::KeepLast(10));
    subscriber_ = rclcpp::Node::create_subscription<ControlMessage>("control_message", ctrl_msg_qos, std::bind(&ODriveCanNode::subscriber_callback, this, _1));

    rclcpp::QoS srv_qos(rclcpp::KeepLast(10));
    service_ = rclcpp::Node::create_service<AxisState>("request_axis_state", std::bind(&ODriveCanNode::service_callback, this, _1, _2), srv_qos.get_rmw_qos_profile());

    // CUSTOM CODE START

    // We set this to only keep the last 10 because with KeepAll, repeatedly creating subscribers will cause the thread to become overloaded and crash
    rclcpp::QoS odrv_advanced_stat_qos(rclcpp::KeepLast(10));

    // Creates the publisher for the ODriveStatusAdvanced
    odrv_advanced_publisher_ = rclcpp::Node::create_publisher<ODriveStatusAdvanced>("odrive_status_advanced", odrv_advanced_stat_qos);

    
    //Creates the subscriber for the ControlVelocityGains messages
    rclcpp::QoS vel_gains_subscriber_qos(rclcpp::KeepLast(10));
    vel_gains_subscriber_ = rclcpp::Node::create_subscription<ControlVelocityGains>("control_vel_gains", vel_gains_subscriber_qos, std::bind(&ODriveCanNode::control_vel_gains_callback, this, _1));

    //Creates the subscriber for the ControlVelocityGains messages
    rclcpp::QoS pos_gains_subscriber_qos(rclcpp::KeepLast(10));
    pos_gains_subscriber_ = rclcpp::Node::create_subscription<ControlPositionGain>("control_pos_gain", pos_gains_subscriber_qos, std::bind(&ODriveCanNode::control_pos_gains_callback, this, _1));

    //Creates the subscriber for the ControlTrajVelLim messages
    rclcpp::QoS control_traj_vel_lim_qos(rclcpp::KeepLast(10));
    control_traj_vel_lim_subscriber_ = rclcpp::Node::create_subscription<ControlTrajVelLim>("control_traj_vel_lim", control_traj_vel_lim_qos, std::bind(&ODriveCanNode::control_traj_vel_lim_callback, this, _1));



    //Creates the subscriber for the ControlTrajAccelLims messages
    rclcpp::QoS control_traj_accel_lims_qos(rclcpp::KeepLast(10));
    control_traj_accel_lims_subscriber_ = rclcpp::Node::create_subscription<ControlTrajAccelLims>("control_traj_accel_lims", control_traj_accel_lims_qos, std::bind(&ODriveCanNode::control_traj_accel_lims_callback, this, _1));



    //Creates the subscriber for the RebootMessage messages
    rclcpp::QoS reboot_message_subscriber_qos(rclcpp::KeepLast(10));
    reboot_msg_subscriber_ = rclcpp::Node::create_subscription<RebootMessage>("reboot_message", reboot_message_subscriber_qos, std::bind(&ODriveCanNode::reboot_message_callback, this, _1));


    //Creates the service for read and writing to arbitrary values
    rclcpp::QoS estop_srv_qos(rclcpp::KeepLast(10));
    estop_service_ = rclcpp::Node::create_service<Estop>("estop", std::bind(&ODriveCanNode::estop_service_callback, this, _1, _2), estop_srv_qos.get_rmw_qos_profile());

    
    //Creates the service for read and writing to arbitrary values
    rclcpp::QoS clear_errors_srv_qos(rclcpp::KeepLast(10));
    clear_errors_service_ = rclcpp::Node::create_service<ClearErrors>("clear_errors", std::bind(&ODriveCanNode::clear_errors_service_callback, this, _1, _2), clear_errors_srv_qos.get_rmw_qos_profile());


   
    //Creates the service for read and writing to arbitrary values
    rclcpp::QoS value_access_srv_qos(rclcpp::KeepLast(10));
    value_access_service_ = rclcpp::Node::create_service<ValueAccess>("access_value", std::bind(&ODriveCanNode::value_access_service_callback, this, _1, _2), value_access_srv_qos.get_rmw_qos_profile());

    
    odrv_advanced_received_powers_ = false;




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


    

    bool set_config_correctly = settingsFromConfig();

    return true;
}

void ODriveCanNode::recv_callback(const can_frame& frame) {
    // This is what is called whenever we receive a frame from the CAN



    if(((frame.can_id >> 5) & 0x3F) != node_id_) return;

    // checks the lower 5 bits of the can_id which store the cmd_id
    // We check which command id it matches to decide what data the CAN has sent to us
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
            // This gets the needed values from 
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

        case CmdId::kGetPowers: {
            if (!verify_length("kGetPowers", 8, frame.can_dlc)) break;
             // CUSTOM CODE START
            std::lock_guard<std::mutex> guard1(odrv_advanced_stat_mutex_);
            odrv_advanced_stat_.electrical_power    = read_le<float>(frame.data + 0);
            odrv_advanced_stat_.mechanical_power    = read_le<float>(frame.data + 4);

            odrv_advanced_received_powers_ = true;
            // CUSTOM CODE END

            break;
        }

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

            if (odrv_advanced_received_powers_ == true){
                odrv_advanced_publisher_->publish(odrv_advanced_stat_);
                odrv_advanced_pub_flag_ = 0;
                odrv_advanced_ctrl_pub_flag_ = 0;
                odrv_advanced_received_powers_ = 0;
            }
            
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


void ODriveCanNode::control_vel_gains_callback(const odrive_can::msg::ControlVelocityGains::SharedPtr msg) {

    RCLCPP_INFO(rclcpp::Node::get_logger(), "Velocity gains callback called");
    RCLCPP_INFO(rclcpp::Node::get_logger(), "vel gain: %f", msg->vel_gain);
    RCLCPP_INFO(rclcpp::Node::get_logger(), "vel integrator gain: %f", msg->vel_integrator_gain);


    // This will used the set_vel_gains protocol to set the vel_gain and vel_integrator_gain
    struct can_frame frame;
    frame.can_id = node_id_ << 5 | kSetVelGains;
    {
        std::lock_guard<std::mutex> guard(vel_gains_msg_mutex_);
        write_le<float>(msg->vel_gain, frame.data);
        write_le<float>(msg->vel_integrator_gain,   frame.data + 4);
    }
    frame.can_dlc = 8;
    can_intf_.send_can_frame(frame);


    // this will send a CAN message to set the vel_integrator_limit value to the one specified in the message
    struct can_frame vel_int_limit_frame;
    vel_int_limit_frame.can_id = node_id_ << 5 | kRxSdo;
    {
        write_le<uint8_t>(1, vel_int_limit_frame.data);
        write_le<uint16_t>(403, vel_int_limit_frame.data + 1);
        write_le<float>(msg->vel_integrator_limit,   vel_int_limit_frame.data + 4);
    }
    vel_int_limit_frame.can_dlc = 8;
    can_intf_.send_can_frame(vel_int_limit_frame);


    // this will send a CAN message to set the vel_limit value to the one specified in the message
    struct can_frame vel_limit_frame;
    vel_limit_frame.can_id = node_id_ << 5 | kRxSdo;
    {
        write_le<uint8_t>(1, vel_limit_frame.data);
        write_le<uint16_t>(404, vel_limit_frame.data + 1);
        write_le<float>(msg->vel_limit,   vel_limit_frame.data + 4);
    }
    vel_limit_frame.can_dlc = 8;
    can_intf_.send_can_frame(vel_limit_frame);



    RCLCPP_INFO(rclcpp::Node::get_logger(), "Velocity Gains Updated!!!");
    
    
}

void ODriveCanNode::control_pos_gains_callback(const odrive_can::msg::ControlPositionGain::SharedPtr msg) {
    struct can_frame frame;
    frame.can_id = node_id_ << 5 | kSetPosGain;
    {
        write_le<float>(msg->pos_gain, frame.data);
    }
    frame.can_dlc = 8;
    can_intf_.send_can_frame(frame);
}

void ODriveCanNode::control_traj_vel_lim_callback(const odrive_can::msg::ControlTrajVelLim::SharedPtr msg) {
    struct can_frame frame;
    frame.can_id = node_id_ << 5 | kSetTrajVelLimit;
    {
        write_le<float>(msg->traj_vel_limit, frame.data);
    }
    frame.can_dlc = 8;
    can_intf_.send_can_frame(frame);
}


void ODriveCanNode::control_traj_accel_lims_callback(const odrive_can::msg::ControlTrajAccelLims::SharedPtr msg) {
    RCLCPP_INFO(rclcpp::Node::get_logger(), "control traj accel callback started");
    struct can_frame frame;
    frame.can_id = node_id_ << 5 | kSetTrajAccelLimit;
    {
        write_le<float>(msg->traj_accel_limit, frame.data);
        write_le<float>(msg->traj_decel_limit, frame.data + 4);
    }
    frame.can_dlc = 8;
    can_intf_.send_can_frame(frame);
}


void ODriveCanNode::reboot_message_callback(const odrive_can::msg::RebootMessage::SharedPtr msg){
    RCLCPP_INFO(rclcpp::Node::get_logger(), "Reboot message callback was initiated %d", msg->action);

    struct can_frame frame;
    frame.can_id = node_id_ << 5 | kReboot;
    {
        write_le<uint8_t>(msg->action, frame.data);
        
    }
    frame.can_dlc = 8;
    can_intf_.send_can_frame(frame);

    RCLCPP_INFO(rclcpp::Node::get_logger(), "Reboot message callback was SENT");
}


// CUSTOM CODE END


// CUSTOM CODE START



// void ODriveCanNode::load_config(){
//     RCLCPP_INFO(rclcpp::Node::get_logger(), "Loading config values from file");
    

//     RCLCPP_INFO(rclcpp::Node::get_logger(), "Loaded config files");
// }


void ODriveCanNode::estop_service_callback(const std::shared_ptr<Estop::Request> request, std::shared_ptr<Estop::Response> response) {
    
    
    {
        std::unique_lock<std::mutex> guard(estop_mutex_);


        Estop::Response current_response = Estop::Response();
    
        // Sending the estop signal via can bus

        struct can_frame frame;
        frame.can_id = node_id_ << 5 | kEstop;
        frame.can_dlc = 8;
        can_intf_.send_can_frame(frame);
      
        current_response.estopped_system = true;

        *response = current_response;

    }
        
}

void ODriveCanNode::clear_errors_service_callback(const std::shared_ptr<ClearErrors::Request> request, std::shared_ptr<ClearErrors::Response> response) {
    
        ClearErrors::Response current_response = ClearErrors::Response();
    
        // Sending the clear errors signal via can bus

        struct can_frame frame;
        frame.can_id = node_id_ << 5 | kClearErrors;

        {
            write_le<uint8_t>(1, frame.data);
        }
        
        frame.can_dlc = 8;
        can_intf_.send_can_frame(frame);
      
        current_response.cleared_errors = true;

        *response = current_response;
        
}





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


// CUSTOM CODE START

bool ODriveCanNode::settingsFromConfig(){
    
        // Trying to set all of the odrive settings from a config file
        // return true if you succeed and false if there is an error

        RCLCPP_INFO(this->get_logger(), "LOADING odrive config values");

        std::map<std::string, int> bool_parameter_map;
        bool_parameter_map["axis0.config.enable_watchdog"] = 219;
        bool_parameter_map["axis0.controller.config.enable_torque_mode_vel_limit"] = 394;

        std::map<std::string, int> float_parameter_map;

        float_parameter_map["config.max_regen_current"] = 139;
        float_parameter_map["config.dc_bus_undervoltage_trip_level"] = 140;
        float_parameter_map["config.dc_bus_overvoltage_trip_level"] = 141;
        float_parameter_map["config.dc_max_positive_current"] = 142;
        float_parameter_map["config.dc_max_negative_current"] = 143;
        float_parameter_map["axis0.config.watchdog_timeout"] = 218; 

        float_parameter_map["axis0.config.I_bus_hard_min"] = 261;
        float_parameter_map["axis0.config.I_bus_hard_max"] = 262;
        float_parameter_map["axis0.config.I_bus_soft_min"] = 263;
        float_parameter_map["axis0.config.I_bus_soft_max"] = 264;
        float_parameter_map["axis0.config.P_bus_soft_min"] = 265;
        float_parameter_map["axis0.config.P_bus_soft_max"] = 266;


        
        float_parameter_map["axis0.config.torque_soft_min"] = 267;
        float_parameter_map["axis0.config.torque_soft_max"] = 268;
        float_parameter_map["axis0.config.motor.current_soft_max"] = 287;
        float_parameter_map["axis0.config.motor.current_hard_max"] = 288;
        float_parameter_map["axis0.controller.config.inertia"] = 416;
        float_parameter_map["axis0.controller.config.spinout_mechanical_power_threshold"] = 420;
        float_parameter_map["axis0.controller.config.spinout_electrical_power_threshold"] = 421;

        std::map<std::string, int> int32_parameter_map;

        std::map<std::string, int> uint64_parameter_map;

        std::map<std::string, int> uint32_parameter_map;

        uint32_parameter_map["axis0.config.can.heartbeat_msg_rate_ms"] = 249;
        uint32_parameter_map["axis0.config.can.encoder_msg_rate_ms"] = 250;
        uint32_parameter_map["axis0.config.can.iq_msg_rate_ms"] = 251;
        uint32_parameter_map["axis0.config.can.error_msg_rate_ms"] = 252;
        uint32_parameter_map["axis0.config.can.temperature_msg_rate_ms"] = 253;
        uint32_parameter_map["axis0.config.can.bus_voltage_msg_rate_ms"] = 254;
        uint32_parameter_map["axis0.config.can.torques_msg_rate_ms"] = 255;
        uint32_parameter_map["axis0.config.can.powers_msg_rate_ms"] = 256;
        
        
        std::map<std::string, int> uint16_parameter_map;

        std::map<std::string, int> uint8_parameter_map;





        // Loop through the list of float parameter names and for each of them call setFloatParameter

        for (auto it = bool_parameter_map.begin(); it != bool_parameter_map.end(); ++it) {
            try{

                setParameter(it->first,it->second, 0);

             } catch (const std::runtime_error& e){
            
            RCLCPP_ERROR(this->get_logger(), "FAILED TO LOAD ALL bool CONFIG VALUES FOR ODRIVE");
            }
        }


        for (auto it = float_parameter_map.begin(); it != float_parameter_map.end(); ++it) {
            try{

                setParameter(it->first,it->second, 1);

             } catch (const std::runtime_error& e){
            
            RCLCPP_ERROR(this->get_logger(), "FAILED TO LOAD ALL FLOAT CONFIG VALUES FOR ODRIVE");
            }
        }



        for (auto it = int32_parameter_map.begin(); it != int32_parameter_map.end(); ++it) {
            try{

                setParameter(it->first,it->second, 2);

             } catch (const std::runtime_error& e){
            
            RCLCPP_ERROR(this->get_logger(), "FAILED TO LOAD ALL int32 CONFIG VALUES FOR ODRIVE");
            }
        }



        for (auto it = uint64_parameter_map.begin(); it != uint64_parameter_map.end(); ++it) {
            try{

                setParameter(it->first,it->second, 3);

             } catch (const std::runtime_error& e){
            
            RCLCPP_ERROR(this->get_logger(), "FAILED TO LOAD ALL uint64 CONFIG VALUES FOR ODRIVE");
            }
        }
        


        for (auto it = uint32_parameter_map.begin(); it != uint32_parameter_map.end(); ++it) {
            try{

                setParameter(it->first,it->second, 4);

             } catch (const std::runtime_error& e){
            
            RCLCPP_ERROR(this->get_logger(), "FAILED TO LOAD ALL uint32  CONFIG VALUES FOR ODRIVE");
            }
        }
        



        for (auto it = uint16_parameter_map.begin(); it != uint16_parameter_map.end(); ++it) {
            try{

                setParameter(it->first,it->second, 5);

             } catch (const std::runtime_error& e){
            
            RCLCPP_ERROR(this->get_logger(), "FAILED TO LOAD ALL uint16 CONFIG VALUES FOR ODRIVE");
            }
        }
        




        for (auto it = uint8_parameter_map.begin(); it != uint8_parameter_map.end(); ++it) {
            try{

                setParameter(it->first,it->second, 6);

             } catch (const std::runtime_error& e){
            
            RCLCPP_ERROR(this->get_logger(), "FAILED TO LOAD ALL uint8 CONFIG VALUES FOR ODRIVE");
            }
        }




        RCLCPP_INFO(this->get_logger(), "LOADED odrive config values");

        return true;

    
}


void ODriveCanNode::setParameter(std::string parameter_name, int parameter_endpoint_id, uint32_t datatype_specifier){
    // Function to set a float parameter to a specific value using the CAN bus

    // parameter_name is a string that contains the name of the parameter that has been passed in via the launch file
    // parameter_endpoint_id is an integer that specifies the endpoint where the value should be updated
    // datatype_specifier is an uint32 that specifies what data type we are working with
    // - (0 - bool, 1 - float32, 2 - int32, 3 - uint64, 4 - uint32, 5 - uint16, 6 - uint8)

    switch(datatype_specifier){

        case 0: {
         // bool
            // RCLCPP_DEBUG(rclcpp::Node::get_logger(), "value type was bool");
            this->declare_parameter<bool>(parameter_name, true);
            break;
        }
        case 1: {
            // float32
            // RCLCPP_DEBUG(rclcpp::Node::get_logger(), "value type was float32");
            this->declare_parameter<float>(parameter_name, 0.0);
            break;
        }
        case 2: {
            // int32
            // RCLCPP_DEBUG(rclcpp::Node::get_logger(), "value type was int32");
            this->declare_parameter<int>(parameter_name, 0);
            break;
        }
        case 3: {
            // int32
            // RCLCPP_DEBUG(rclcpp::Node::get_logger(), "value type was int32");
            this->declare_parameter<int>(parameter_name, 0);
            break;
        }
        case 4: {
            // int32
            // RCLCPP_DEBUG(rclcpp::Node::get_logger(), "value type was int32");
            this->declare_parameter<int>(parameter_name, 0);
            break;
        }
        case 5: {
            // int32
            // RCLCPP_DEBUG(rclcpp::Node::get_logger(), "value type was int32");
            this->declare_parameter<int>(parameter_name, 0);
            break;
        }
        case 6: {
            // int32
            // RCLCPP_DEBUG(rclcpp::Node::get_logger(), "value type was int32");
            this->declare_parameter<int>(parameter_name, 0);
            break;
        }
        default: 
            
            RCLCPP_ERROR(rclcpp::Node::get_logger(), "unsupported data type specified: %d", datatype_specifier);
            return;

    }

   

    // Check if the parameter has been passed in
    //  - If so then set the parameter to the passed in value
    //  - If not then don't set the parameter
    if (this->has_parameter(parameter_name)) {
        // Retrieve the parameter value
        
        // RCLCPP_INFO(this->get_logger(), "%s should be loaded to be %f ",parameter_name.c_str(), endpoint_id);
        // This will set the endpoint of 139 to be the passed in value through a CAN bus message
    
        struct can_frame frame;
        frame.can_id = node_id_ << 5 | kRxSdo;
        {
            write_le<uint8_t>(1, frame.data);
            write_le<uint16_t>(parameter_endpoint_id, frame.data + 1);

            switch(datatype_specifier){

                case 0: {
                 // bool
                    // RCLCPP_DEBUG(rclcpp::Node::get_logger(), "value type was bool");
                    bool endpoint_id = this->get_parameter(parameter_name).as_bool();
                    write_le<bool>(endpoint_id,   frame.data + 4);
                    break;
                }
                case 1: {
                    // float32
                    // RCLCPP_DEBUG(rclcpp::Node::get_logger(), "value type was float32");
                    float endpoint_id = this->get_parameter(parameter_name).as_double();
                    write_le<float>(endpoint_id,   frame.data + 4);
                    break;
                }
                case 2: {
                    // int32
                    // RCLCPP_DEBUG(rclcpp::Node::get_logger(), "value type was int32");
                    int32_t endpoint_id = this->get_parameter(parameter_name).as_int();
                    write_le<int32_t>(endpoint_id,   frame.data + 4);
                    break;
                }
                case 3: {
                    // uint64
                    // RCLCPP_DEBUG(rclcpp::Node::get_logger(), "value type was uint64");
                    uint64_t endpoint_id = this->get_parameter(parameter_name).as_int();
                    write_le<uint64_t>(endpoint_id,   frame.data + 4);
                    break;
                }    
                case 4: {
                    // uint32
                    // RCLCPP_DEBUG(rclcpp::Node::get_logger(), "value type was uint32");
                    uint32_t endpoint_id = this->get_parameter(parameter_name).as_int();
                    write_le<uint32_t>(endpoint_id,   frame.data + 4);
                    break;
                }
                case 5: {
                    // uint16
                    // RCLCPP_DEBUG(rclcpp::Node::get_logger(), "value type was uint16");
                    uint16_t endpoint_id = this->get_parameter(parameter_name).as_int();
                    write_le<uint16_t>(endpoint_id,   frame.data + 4);
                    break;
                }
                case 6: {
                    // uint8
                    // RCLCPP_DEBUG(rclcpp::Node::get_logger(), "value type was uint8");
                    uint8_t endpoint_id = this->get_parameter(parameter_name).as_int();
                    write_le<uint8_t>(endpoint_id,   frame.data + 4);
                    break;
                }
                default: 

                    RCLCPP_ERROR(rclcpp::Node::get_logger(), "Error in sending value to can unsupported data type specified: %d", datatype_specifier);
                    return;

            }

            
        }
        frame.can_dlc = 8;
        can_intf_.send_can_frame(frame);

    } else {
        RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not set. Using default value.", parameter_name.c_str());
    }

}

// CUSTOM CODE END

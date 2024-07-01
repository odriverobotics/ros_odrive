#ifndef ODRIVE_CAN_NODE_HPP
#define ODRIVE_CAN_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "odrive_can/msg/o_drive_status.hpp"
#include "odrive_can/msg/controller_status.hpp"
#include "odrive_can/msg/control_message.hpp"
#include "odrive_can/srv/axis_state.hpp"
#include "socket_can.hpp"

// CUSTOM CODE START
#include "odrive_can/msg/o_drive_status_advanced.hpp"
#include "odrive_can/msg/control_velocity_gains.hpp"
#include "odrive_can/srv/value_access.hpp"
#include <string>
#include <vector>
// CUSTOM CODE END

#include <mutex>
#include <condition_variable>
#include <array>
#include <algorithm>
#include <linux/can.h>
#include <linux/can/raw.h>

using std::placeholders::_1;
using std::placeholders::_2;

using ODriveStatus = odrive_can::msg::ODriveStatus;
using ControllerStatus = odrive_can::msg::ControllerStatus;
using ControlMessage = odrive_can::msg::ControlMessage;

// CUSTOM CODE START

using ODriveStatusAdvanced = odrive_can::msg::ODriveStatusAdvanced;

using ControlVelocityGains = odrive_can::msg::ControlVelocityGains;

using ValueAccess = odrive_can::srv::ValueAccess;


// CUSTOM CODE END

using AxisState = odrive_can::srv::AxisState;

class ODriveCanNode : public rclcpp::Node {
public:
    ODriveCanNode(const std::string& node_name);
    bool init(EpollEventLoop* event_loop); 
    void deinit();
private:
    void recv_callback(const can_frame& frame);
    void subscriber_callback(const ControlMessage::SharedPtr msg);
    void service_callback(const std::shared_ptr<AxisState::Request> request, std::shared_ptr<AxisState::Response> response);
    void request_state_callback();
    void ctrl_msg_callback();
    // CUSTOM CODE START
    void control_gains_callback(const odrive_can::msg::ControlVelocityGains::SharedPtr msg);
    
    void value_access_service_callback(const std::shared_ptr<ValueAccess::Request> request, std::shared_ptr<ValueAccess::Response> response);

     
    // CUSTOM CODE END
    inline bool verify_length(const std::string&name, uint8_t expected, uint8_t length);
    
    uint16_t node_id_;
    SocketCanIntf can_intf_ = SocketCanIntf();
    
    short int ctrl_pub_flag_ = 0;
    std::mutex ctrl_stat_mutex_;
    ControllerStatus ctrl_stat_ = ControllerStatus();
    rclcpp::Publisher<ControllerStatus>::SharedPtr ctrl_publisher_;
    
    short int odrv_pub_flag_ = 0;
    std::mutex odrv_stat_mutex_;
    ODriveStatus odrv_stat_ = ODriveStatus();
    rclcpp::Publisher<ODriveStatus>::SharedPtr odrv_publisher_;

    EpollEvent sub_evt_;
    std::mutex ctrl_msg_mutex_;
    ControlMessage ctrl_msg_ = ControlMessage();
    rclcpp::Subscription<ControlMessage>::SharedPtr subscriber_;
    EpollEvent srv_evt_;
    uint32_t axis_state_;
    std::mutex axis_state_mutex_;
    std::condition_variable fresh_heartbeat_;
    rclcpp::Service<AxisState>::SharedPtr service_;

    //CUSTOM CODE START
    std::mutex gains_msg_mutex_;
    ControlVelocityGains gains_msg_ = ControlVelocityGains();
    rclcpp::Subscription<ControlVelocityGains>::SharedPtr gains_subscriber_;



    EpollEvent value_access_srv_evt_;
    uint32_t value_access_datatype_specifier_;
    ValueAccess::Response value_access_reponse_;
    std::condition_variable fresh_TxSdo_;
    bool received_TxSdo_;
    std::mutex value_access_mutex_;
    rclcpp::Service<ValueAccess>::SharedPtr value_access_service_;
   


    AxisState::Response value_access_response;




    //CUSTOM CODE END




    // CUSTOM CODE START
    short int odrv_advanced_ctrl_pub_flag_ = 0;
    short int odrv_advanced_pub_flag_ = 0;
    std::mutex odrv_advanced_stat_mutex_;
    ODriveStatusAdvanced odrv_advanced_stat_ = ODriveStatusAdvanced();
    rclcpp::Publisher<ODriveStatusAdvanced>::SharedPtr odrv_advanced_publisher_;

    // CUSTOM CODE END


    // CUSTOM CODE START

    bool settingsFromConfig();

    void setFloatParameter(std::string parameter_name, int parameter_endpoint_id);

    // CUSTOM CODE END

};

#endif // ODRIVE_CAN_NODE_HPP

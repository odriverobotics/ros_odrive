
#include "can_helpers.hpp"
#include "can_simple_messages.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "odrive_enums.h"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "socket_can.hpp"

namespace odrive_ros2_control {

class Axis;

class ODriveHardwareInterface final : public hardware_interface::SystemInterface {
public:
    using return_type = hardware_interface::return_type;
    using State = rclcpp_lifecycle::State;

    CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
    CallbackReturn on_configure(const State& previous_state) override;
    CallbackReturn on_cleanup(const State& previous_state) override;
    CallbackReturn on_activate(const State& previous_state) override;
    CallbackReturn on_deactivate(const State& previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    return_type perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces
    ) override;

    return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;
    return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;

private:
    void on_can_msg(const can_frame& frame);

    EpollEventLoop event_loop_;
    std::vector<Axis> axes_;
    std::string can_intf_name_;
    SocketCanIntf can_intf_;
    rclcpp::Time timestamp_;
};

struct Axis {
    Axis(SocketCanIntf* can_intf, uint32_t node_id) : can_intf_(can_intf), node_id_(node_id) {}

    void on_can_msg(const rclcpp::Time& timestamp, const can_frame& frame);

    void on_can_msg();

    SocketCanIntf* can_intf_;
    uint32_t node_id_;

    // Commands (ros2_control => ODrives)
    double pos_setpoint_ = 0.0f; // [rad]
    double vel_setpoint_ = 0.0f; // [rad/s]
    double torque_setpoint_ = 0.0f; // [Nm]

    // State (ODrives => ros2_control)
    // rclcpp::Time encoder_estimates_timestamp_;
    // uint32_t axis_error_ = 0;
    // uint8_t axis_state_ = 0;
    // uint8_t procedure_result_ = 0;
    // uint8_t trajectory_done_flag_ = 0;
    double pos_estimate_ = NAN; // [rad]
    double vel_estimate_ = NAN; // [rad/s]
    // double iq_setpoint_ = NAN;
    // double iq_measured_ = NAN;
    double torque_target_ = NAN; // [Nm]
    double torque_estimate_ = NAN; // [Nm]
    // uint32_t active_errors_ = 0;
    // uint32_t disarm_reason_ = 0;
    // double fet_temperature_ = NAN;
    // double motor_temperature_ = NAN;
    // double bus_voltage_ = NAN;
    // double bus_current_ = NAN;

    // Indicates which controller inputs are enabled. This is configured by the
    // controller that sits on top of this hardware interface. Multiple inputs
    // can be enabled at the same time, in this case the non-primary inputs are
    // used as feedforward terms.
    // This implicitly defines the ODrive's control mode.
    bool pos_input_enabled_ = false;
    bool vel_input_enabled_ = false;
    bool torque_input_enabled_ = false;

    template <typename T>
    void send(const T& msg) {
        struct can_frame frame;
        frame.can_id = node_id_ << 5 | msg.cmd_id;
        frame.can_dlc = msg.msg_length;
        msg.encode_buf(frame.data);

        can_intf_->send_can_frame(frame);
    }
};

} // namespace odrive_ros2_control

using namespace odrive_ros2_control;

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

CallbackReturn ODriveHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    can_intf_name_ = info_.hardware_parameters["can"];

    for (auto& joint : info_.joints) {
        axes_.emplace_back(&can_intf_, std::stoi(joint.parameters.at("node_id")));
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_configure(const State&) {
    if (!can_intf_.init(can_intf_name_, &event_loop_, std::bind(&ODriveHardwareInterface::on_can_msg, this, _1))) {
        RCLCPP_ERROR(rclcpp::get_logger("ODriveHardwareInterface"), "Failed to initialize SocketCAN on %s", can_intf_name_.c_str());
        return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Initialized SocketCAN on %s", can_intf_name_.c_str());
    return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_cleanup(const State&) {
    can_intf_.deinit();
    return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_activate(const State&) {
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "activating ODrives...");

    // This can be called several seconds before the controller finishes starting.
    // Therefore we enable the ODrives only in perform_command_mode_switch().
    return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_deactivate(const State&) {
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "deactivating ODrives...");

    for (auto& axis : axes_) {
        Set_Axis_State_msg_t msg;
        msg.Axis_Requested_State = AXIS_STATE_IDLE;
        axis.send(msg);
    }

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ODriveHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_EFFORT,
            &axes_[i].torque_target_
        ));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY,
            &axes_[i].vel_estimate_
        ));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &axes_[i].pos_estimate_
        ));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ODriveHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_EFFORT,
            &axes_[i].torque_setpoint_
        ));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY,
            &axes_[i].vel_setpoint_
        ));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &axes_[i].pos_setpoint_
        ));
    }

    return command_interfaces;
}

return_type ODriveHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces
) {
    for (size_t i = 0; i < axes_.size(); ++i) {
        Axis& axis = axes_[i];
        std::array<std::pair<std::string, bool*>, 3> interfaces = {
            {{info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION, &axis.pos_input_enabled_},
             {info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY, &axis.vel_input_enabled_},
             {info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT, &axis.torque_input_enabled_}}
        };

        bool mode_switch = false;

        for (const std::string& key : stop_interfaces) {
            for (auto& kv : interfaces) {
                if (kv.first == key) {
                    *kv.second = false;
                    mode_switch = true;
                }
            }
        }

        for (const std::string& key : start_interfaces) {
            for (auto& kv : interfaces) {
                if (kv.first == key) {
                    *kv.second = true;
                    mode_switch = true;
                }
            }
        }

        if (mode_switch) {
            Set_Controller_Mode_msg_t msg;
            if (axis.pos_input_enabled_) {
                RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Setting %s to position control", info_.joints[i].name.c_str());
                msg.Control_Mode = CONTROL_MODE_POSITION_CONTROL;
                msg.Input_Mode = INPUT_MODE_PASSTHROUGH;
            } else if (axis.vel_input_enabled_) {
                RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Setting %s to velocity control", info_.joints[i].name.c_str());
                msg.Control_Mode = CONTROL_MODE_VELOCITY_CONTROL;
                msg.Input_Mode = INPUT_MODE_PASSTHROUGH;
            } else {
                RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Setting %s to torque control", info_.joints[i].name.c_str());
                msg.Control_Mode = CONTROL_MODE_TORQUE_CONTROL;
                msg.Input_Mode = INPUT_MODE_PASSTHROUGH;
            }

            bool any_enabled = axis.pos_input_enabled_ || axis.vel_input_enabled_ || axis.torque_input_enabled_;

            if (any_enabled) {
                axis.send(msg); // Set control mode
            }

            // Set axis state
            Clear_Errors_msg_t msg1;
            msg1.Identify = 0;
            axis.send(msg1);

            // Set axis state
            Set_Axis_State_msg_t msg2;
            msg2.Axis_Requested_State = any_enabled ? AXIS_STATE_CLOSED_LOOP_CONTROL : AXIS_STATE_IDLE;
            axis.send(msg2);
        }
    }

    return return_type::OK;
}

return_type ODriveHardwareInterface::read(const rclcpp::Time& timestamp, const rclcpp::Duration&) {
    timestamp_ = timestamp;

    while (can_intf_.read_nonblocking()) {
        // repeat until CAN interface has no more messages
    }

    return return_type::OK;
}

return_type ODriveHardwareInterface::write(const rclcpp::Time&, const rclcpp::Duration&) {
    for (auto& axis : axes_) {
        // Send the CAN message that fits the set of enabled setpoints
        if (axis.pos_input_enabled_) {
            Set_Input_Pos_msg_t msg;
            msg.Input_Pos = axis.pos_setpoint_ / (2 * M_PI);
            msg.Vel_FF = axis.vel_input_enabled_ ? (axis.vel_setpoint_  / (2 * M_PI)) : 0.0f;
            msg.Torque_FF = axis.torque_input_enabled_ ? axis.torque_setpoint_ : 0.0f;
            axis.send(msg);
        } else if (axis.vel_input_enabled_) {
            Set_Input_Vel_msg_t msg;
            msg.Input_Vel = axis.vel_setpoint_ / (2 * M_PI);
            msg.Input_Torque_FF = axis.torque_input_enabled_ ? axis.torque_setpoint_ : 0.0f;
            axis.send(msg);
        } else if (axis.torque_input_enabled_) {
            Set_Input_Torque_msg_t msg;
            msg.Input_Torque = axis.torque_setpoint_;
            axis.send(msg);
        } else {
            // no control enabled - don't send any setpoint
        }
    }

    return return_type::OK;
}

void ODriveHardwareInterface::on_can_msg(const can_frame& frame) {
    for (auto& axis : axes_) {
        if ((frame.can_id >> 5) == axis.node_id_) {
            axis.on_can_msg(timestamp_, frame);
        }
    }
}

void Axis::on_can_msg(const rclcpp::Time&, const can_frame& frame) {
    uint8_t cmd = frame.can_id & 0x1f;

    auto try_decode = [&]<typename TMsg>(TMsg& msg) {
        if (frame.can_dlc < Get_Encoder_Estimates_msg_t::msg_length) {
            RCLCPP_WARN(rclcpp::get_logger("ODriveHardwareInterface"), "message %d too short", cmd);
            return false;
        }
        msg.decode_buf(frame.data);
        return true;
    };

    switch (cmd) {
        case Get_Encoder_Estimates_msg_t::cmd_id: {
            if (Get_Encoder_Estimates_msg_t msg; try_decode(msg)) {
                pos_estimate_ = msg.Pos_Estimate * (2 * M_PI);
                vel_estimate_ = msg.Vel_Estimate * (2 * M_PI);
            }
        } break;
        case Get_Torques_msg_t::cmd_id: {
            if (Get_Torques_msg_t msg; try_decode(msg)) {
                torque_target_ = msg.Torque_Target;
                torque_estimate_ = msg.Torque_Estimate;
            }
        } break;
            // silently ignore unimplemented command IDs
    }
}

PLUGINLIB_EXPORT_CLASS(odrive_ros2_control::ODriveHardwareInterface, hardware_interface::SystemInterface)

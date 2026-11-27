// Checks which setpoint messages write() sends, using a fake CAN interface.
// A setpoint must not go out until the controller has written it.

// The hardware interface class is only declared in its .cpp file.
#include "../src/odrive_hardware_interface.cpp"

#include "fake_socket_can.hpp"

#include <gtest/gtest.h>

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace {

constexpr uint32_t kNodeId = 3;

class SetpointTest : public ::testing::Test {
protected:
    void SetUp() override {
        hardware_interface::ComponentInfo joint;
        joint.name = "joint0";
        joint.type = "joint";
        joint.parameters["node_id"] = std::to_string(kNodeId);

        hardware_interface::HardwareInfo info;
        info.name = "odrive";
        info.type = "system";
        info.hardware_parameters["can"] = "fake0";
        info.joints.push_back(joint);

        ASSERT_EQ(hw_.on_init(info), CallbackReturn::SUCCESS);
        ASSERT_EQ(hw_.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
        ASSERT_EQ(hw_.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

        for (auto& command : hw_.export_command_interfaces()) {
            commands_.emplace(command.get_name(), std::move(command));
        }
        fake_can_sent_frames().clear();
    }

    void switch_mode(const std::vector<std::string>& start, const std::vector<std::string>& stop = {}) {
        ASSERT_EQ(hw_.perform_command_mode_switch(start, stop), return_type::OK);
        fake_can_sent_frames().clear();
    }

    void command(const std::string& name, double value) {
        (void)commands_.at(name).set_value(value);
    }

    // Runs one write() cycle and returns the setpoint frames it sent.
    std::vector<can_frame> write_cycle() {
        fake_can_sent_frames().clear();
        EXPECT_EQ(hw_.write(rclcpp::Time(), rclcpp::Duration(0, 0)), return_type::OK);
        std::vector<can_frame> frames = fake_can_sent_frames();
        for (const auto& frame : frames) {
            EXPECT_EQ(frame.can_id >> 5, kNodeId);
        }
        return frames;
    }

    template <typename TMsg>
    static std::optional<TMsg> only(const std::vector<can_frame>& frames) {
        if (frames.size() != 1 || (frames[0].can_id & 0x1f) != TMsg::cmd_id) {
            return std::nullopt;
        }
        TMsg msg;
        msg.decode_buf(frames[0].data);
        return msg;
    }

    ODriveHardwareInterface hw_;
    std::map<std::string, hardware_interface::CommandInterface> commands_;
};

const std::string kPos = "joint0/position";
const std::string kVel = "joint0/velocity";
const std::string kEffort = "joint0/effort";

TEST_F(SetpointTest, PositionIsNotSentUntilWritten) {
    switch_mode({kPos});
    EXPECT_TRUE(write_cycle().empty());

    command(kPos, M_PI);
    auto msg = only<Set_Input_Pos_msg_t>(write_cycle());
    ASSERT_TRUE(msg);
    EXPECT_FLOAT_EQ(msg->Input_Pos, 0.5f);
}

TEST_F(SetpointTest, VelocityIsNotSentUntilWritten) {
    switch_mode({kVel});
    EXPECT_TRUE(write_cycle().empty());

    command(kVel, 2 * M_PI);
    auto msg = only<Set_Input_Vel_msg_t>(write_cycle());
    ASSERT_TRUE(msg);
    EXPECT_FLOAT_EQ(msg->Input_Vel, 1.0f);
    EXPECT_FLOAT_EQ(msg->Input_Torque_FF, 0.0f);
}

TEST_F(SetpointTest, TorqueIsNotSentUntilWritten) {
    switch_mode({kEffort});
    EXPECT_TRUE(write_cycle().empty());

    command(kEffort, 0.5);
    auto msg = only<Set_Input_Torque_msg_t>(write_cycle());
    ASSERT_TRUE(msg);
    EXPECT_FLOAT_EQ(msg->Input_Torque, 0.5f);
}

TEST_F(SetpointTest, SwappingVelocityControllersDropsTheOldSetpoint) {
    switch_mode({kVel});
    command(kVel, 2 * M_PI);
    ASSERT_TRUE(only<Set_Input_Vel_msg_t>(write_cycle()));

    // controller_manager stops the old controller's interface and starts the new one's
    switch_mode({kVel}, {kVel});
    EXPECT_TRUE(write_cycle().empty());

    command(kVel, -M_PI);
    auto msg = only<Set_Input_Vel_msg_t>(write_cycle());
    ASSERT_TRUE(msg);
    EXPECT_FLOAT_EQ(msg->Input_Vel, -0.5f);
}

TEST_F(SetpointTest, SwappingTorqueControllersDropsTheOldSetpoint) {
    switch_mode({kEffort});
    command(kEffort, 0.5);
    ASSERT_TRUE(only<Set_Input_Torque_msg_t>(write_cycle()));

    switch_mode({kEffort}, {kEffort});
    EXPECT_TRUE(write_cycle().empty());
}

TEST_F(SetpointTest, PositionVelocityPositionDropsTheOldPositionSetpoint) {
    switch_mode({kPos});
    command(kPos, 7.0);
    ASSERT_TRUE(only<Set_Input_Pos_msg_t>(write_cycle()));

    switch_mode({kVel}, {kPos});
    EXPECT_TRUE(write_cycle().empty());

    switch_mode({kPos}, {kVel});
    EXPECT_TRUE(write_cycle().empty());
}

TEST_F(SetpointTest, UnwrittenFeedforwardIsSentAsZero) {
    switch_mode({kPos, kVel, kEffort});
    command(kPos, M_PI);
    auto pos = only<Set_Input_Pos_msg_t>(write_cycle());
    ASSERT_TRUE(pos);
    EXPECT_FLOAT_EQ(pos->Input_Pos, 0.5f);
    EXPECT_FLOAT_EQ(pos->Vel_FF, 0.0f);
    EXPECT_FLOAT_EQ(pos->Torque_FF, 0.0f);

    switch_mode({kVel, kEffort}, {kPos, kVel, kEffort});
    command(kVel, 2 * M_PI);
    auto vel = only<Set_Input_Vel_msg_t>(write_cycle());
    ASSERT_TRUE(vel);
    EXPECT_FLOAT_EQ(vel->Input_Vel, 1.0f);
    EXPECT_FLOAT_EQ(vel->Input_Torque_FF, 0.0f);
}

TEST_F(SetpointTest, WrittenFeedforwardIsSent) {
    switch_mode({kPos, kVel, kEffort});
    command(kPos, M_PI);
    command(kVel, 2 * M_PI);
    command(kEffort, 0.25);
    auto msg = only<Set_Input_Pos_msg_t>(write_cycle());
    ASSERT_TRUE(msg);
    EXPECT_FLOAT_EQ(msg->Input_Pos, 0.5f);
    // the feedforward fields are int16 with a 0.001 scale
    EXPECT_NEAR(msg->Vel_FF, 1.0f, 2e-3);
    EXPECT_NEAR(msg->Torque_FF, 0.25f, 2e-3);
}

} // namespace

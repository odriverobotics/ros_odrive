#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <thread>

// Exercise the implementation through its public hardware-interface API.
// Only the CAN transport is replaced: no CAN device, permissions or motors needed.
#include "../src/odrive_hardware_interface.cpp"

namespace {
std::deque<can_frame> received_frames;
std::vector<can_frame> sent_frames;
}

bool SocketCanIntf::init(const std::string&, EpollEventLoop*, FrameProcessor processor) {
    frame_processor_ = std::move(processor);
    return true;
}

void SocketCanIntf::deinit() {
    frame_processor_ = {};
    received_frames.clear();
}

bool SocketCanIntf::read_nonblocking() {
    if (received_frames.empty()) return false;
    const auto frame = received_frames.front();
    received_frames.pop_front();
    frame_processor_(frame);
    return true;
}

bool SocketCanIntf::send_can_frame(const can_frame& frame) {
    sent_frames.push_back(frame);
    return true;
}

namespace {

class ODriveFeedbackTest : public ::testing::Test {
protected:
    void SetUp() override {
        received_frames.clear();
        sent_frames.clear();
        hardware_interface::HardwareInfo info;
        info.name = "test_odrives";
        info.type = "system";
        info.hardware_parameters["can"] = "test_transport";
        for (unsigned node = 0; node < 2; ++node) {
            hardware_interface::ComponentInfo joint;
            joint.name = node == 0 ? "left" : "right";
            joint.type = "joint";
            joint.parameters["node_id"] = std::to_string(node);
            info.joints.push_back(joint);
        }
        ASSERT_EQ(hardware_.on_init(info), CallbackReturn::SUCCESS);
        ASSERT_EQ(hardware_.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
        states_ = hardware_.export_state_interfaces();
    }

    void TearDown() override {
        hardware_.on_cleanup(rclcpp_lifecycle::State{});
    }

    template<typename Message>
    can_frame frame(const Message& message, unsigned node = 0) {
        can_frame result{};
        result.can_id = (node << 5) | Message::cmd_id;
        result.can_dlc = Message::msg_length;
        message.encode_buf(result.data);
        return result;
    }

    Heartbeat_msg_t heartbeat(uint32_t errors = 0, uint8_t state = AXIS_STATE_IDLE) {
        Heartbeat_msg_t message{};
        message.Axis_Error = errors;
        message.Axis_State = state;
        message.Procedure_Result = PROCEDURE_RESULT_BUSY;
        message.Trajectory_Done_Flag = 1;
        return message;
    }

    void read(int64_t ros_nanoseconds = 0) {
        ASSERT_EQ(hardware_.read(rclcpp::Time(ros_nanoseconds), rclcpp::Duration(0, 0)),
                  return_type::OK);
    }

    double value(const std::string& field, const std::string& joint = "left") {
        const auto name = joint + "/" + field;
        auto it = std::find_if(states_.begin(), states_.end(), [&](const auto& state) {
            return state.get_name() == name;
        });
        if (it == states_.end()) {
            ADD_FAILURE() << "Missing state interface " << name;
            return std::numeric_limits<double>::quiet_NaN();
        }
        // Preserve Humble support while using the newer handle API when available.
        const auto get_value = []<typename Handle>(const Handle& handle) {
            if constexpr (requires { handle.get_optional(); }) {
                return handle.get_optional().value_or(NAN);
            } else {
                return handle.get_value();
            }
        };
        return get_value(*it);
    }

    const std::vector<std::string> feedback_fields_ = {
        "axis_error", "axis_state", "procedure_result", "trajectory_done_flag",
        "active_errors", "disarm_reason", "heartbeat_age", "error_age"
    };
    ODriveHardwareInterface hardware_;
    std::vector<hardware_interface::StateInterface> states_;
};

TEST_F(ODriveFeedbackTest, FeedbackIsUnknownBeforeReception) {
    EXPECT_EQ(states_.size(), 2u * (3u + feedback_fields_.size()));
    read();
    for (const auto& joint : {"left", "right"}) {
        for (const auto& field : feedback_fields_) EXPECT_TRUE(std::isnan(value(field, joint)));
    }
}

TEST_F(ODriveFeedbackTest, HeartbeatDoesNotInventSplitErrorFeedback) {
    received_frames.push_back(frame(heartbeat(0x80000001u, AXIS_STATE_CLOSED_LOOP_CONTROL)));
    read();
    EXPECT_EQ(value("axis_error"), 0x80000001u);
    EXPECT_EQ(value("axis_state"), AXIS_STATE_CLOSED_LOOP_CONTROL);
    EXPECT_EQ(value("procedure_result"), PROCEDURE_RESULT_BUSY);
    EXPECT_EQ(value("trajectory_done_flag"), 1);
    EXPECT_GE(value("heartbeat_age"), 0);
    EXPECT_TRUE(std::isnan(value("active_errors")));
    EXPECT_TRUE(std::isnan(value("disarm_reason")));
    EXPECT_TRUE(std::isnan(value("error_age")));
}

TEST_F(ODriveFeedbackTest, PreservesFullErrorMasksWithoutHeartbeat) {
    Get_Error_msg_t message{};
    message.Active_Errors = 0xffffffffu;
    message.Disarm_Reason = 0x80000001u;
    received_frames.push_back(frame(message));
    read();
    EXPECT_EQ(value("active_errors"), 4294967295.0);
    EXPECT_EQ(value("disarm_reason"), 2147483649.0);
    EXPECT_GE(value("error_age"), 0);
    EXPECT_TRUE(std::isnan(value("axis_state")));
    EXPECT_TRUE(std::isnan(value("heartbeat_age")));
}

TEST_F(ODriveFeedbackTest, DecodesKnownWirePayloads) {
    received_frames.push_back(can_frame{0x001, 8, 0, 0, 0, {1, 0, 0, 0x80, 8, 1, 1, 0}});
    received_frames.push_back(can_frame{0x003, 8, 0, 0, 0, {1, 0, 0, 0, 0, 0, 0, 0x80}});
    read();
    EXPECT_EQ(value("axis_error"), 2147483649.0);
    EXPECT_EQ(value("axis_state"), AXIS_STATE_CLOSED_LOOP_CONTROL);
    EXPECT_EQ(value("procedure_result"), PROCEDURE_RESULT_BUSY);
    EXPECT_EQ(value("trajectory_done_flag"), 1);
    EXPECT_EQ(value("active_errors"), 1);
    EXPECT_EQ(value("disarm_reason"), 2147483648.0);
}

TEST_F(ODriveFeedbackTest, RetainsHistoricalReasonWhenActiveFaultClears) {
    Get_Error_msg_t error{};
    error.Active_Errors = ODRIVE_ERROR_DC_BUS_UNDER_VOLTAGE;
    error.Disarm_Reason = error.Active_Errors;
    received_frames.push_back(frame(error));
    read();
    EXPECT_EQ(value("active_errors"), ODRIVE_ERROR_DC_BUS_UNDER_VOLTAGE);
    error.Active_Errors = 0;
    received_frames.push_back(frame(error));
    received_frames.push_back(frame(heartbeat(error.Disarm_Reason)));
    read();
    EXPECT_EQ(value("active_errors"), 0);
    EXPECT_EQ(value("disarm_reason"), ODRIVE_ERROR_DC_BUS_UNDER_VOLTAGE);
    EXPECT_EQ(value("axis_error"), ODRIVE_ERROR_DC_BUS_UNDER_VOLTAGE);
    EXPECT_TRUE(sent_frames.empty());
}

TEST_F(ODriveFeedbackTest, AgesIncreaseWithoutMessagesAndIgnoreRosClockJumps) {
    received_frames.push_back(frame(heartbeat()));
    received_frames.push_back(frame(Get_Error_msg_t{}));
    read(10000000000);
    const auto old_heartbeat_age = value("heartbeat_age");
    const auto old_error_age = value("error_age");
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    read(0);
    EXPECT_GT(value("heartbeat_age"), old_heartbeat_age);
    EXPECT_GT(value("error_age"), old_error_age);
    EXPECT_EQ(value("active_errors"), 0);
    EXPECT_TRUE(sent_frames.empty());
}

TEST_F(ODriveFeedbackTest, HeartbeatRefreshDoesNotRefreshErrorAge) {
    received_frames.push_back(frame(heartbeat()));
    received_frames.push_back(frame(Get_Error_msg_t{}));
    read();
    const auto previous_error_age = value("error_age");
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    received_frames.push_back(frame(heartbeat()));
    read();
    EXPECT_GT(value("error_age"), previous_error_age);
    EXPECT_GT(value("error_age"), value("heartbeat_age"));
}

TEST_F(ODriveFeedbackTest, ErrorRefreshDoesNotRefreshHeartbeatAge) {
    received_frames.push_back(frame(heartbeat()));
    received_frames.push_back(frame(Get_Error_msg_t{}));
    read();
    const auto previous_heartbeat_age = value("heartbeat_age");
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    received_frames.push_back(frame(Get_Error_msg_t{}));
    read();
    EXPECT_GT(value("heartbeat_age"), previous_heartbeat_age);
    EXPECT_GT(value("heartbeat_age"), value("error_age"));
}

TEST_F(ODriveFeedbackTest, ShortMessagesDoNotInitializeFeedback) {
    auto short_heartbeat = frame(heartbeat());
    auto short_error = frame(Get_Error_msg_t{});
    short_heartbeat.can_dlc = Heartbeat_msg_t::msg_length - 1;
    short_error.can_dlc = Get_Error_msg_t::msg_length - 1;
    received_frames.push_back(short_heartbeat);
    received_frames.push_back(short_error);
    read();
    for (const auto& field : feedback_fields_) EXPECT_TRUE(std::isnan(value(field)));
}

TEST_F(ODriveFeedbackTest, ShortMessagesPreserveValuesAndDoNotRefreshAges) {
    received_frames.push_back(frame(heartbeat(0x1234)));
    Get_Error_msg_t error{};
    error.Active_Errors = 0x5678;
    error.Disarm_Reason = 0x9abc;
    received_frames.push_back(frame(error));
    read();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    auto short_heartbeat = frame(heartbeat());
    auto short_error = frame(Get_Error_msg_t{});
    short_heartbeat.can_dlc = short_error.can_dlc = 0;
    received_frames.push_back(short_heartbeat);
    received_frames.push_back(short_error);
    read();
    EXPECT_EQ(value("axis_error"), 0x1234);
    EXPECT_EQ(value("active_errors"), 0x5678);
    EXPECT_EQ(value("disarm_reason"), 0x9abc);
    EXPECT_GE(value("heartbeat_age"), 0.002);
    EXPECT_GE(value("error_age"), 0.002);
}

TEST_F(ODriveFeedbackTest, RoutesFeedbackPerNodeAndIgnoresOtherNodes) {
    received_frames.push_back(frame(heartbeat(1), 0));
    received_frames.push_back(frame(heartbeat(2), 1));
    received_frames.push_back(frame(heartbeat(3), 2));
    read();
    EXPECT_EQ(value("axis_error", "left"), 1);
    EXPECT_EQ(value("axis_error", "right"), 2);
    EXPECT_TRUE(std::isnan(value("error_age", "right")));
}

TEST_F(ODriveFeedbackTest, ReconfigureForgetsFeedbackAndAges) {
    received_frames.push_back(frame(heartbeat()));
    received_frames.push_back(frame(Get_Error_msg_t{}));
    read();
    ASSERT_FALSE(std::isnan(value("heartbeat_age")));
    ASSERT_EQ(hardware_.on_cleanup(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
    ASSERT_EQ(hardware_.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
    read();
    for (const auto& field : feedback_fields_) EXPECT_TRUE(std::isnan(value(field)));
}

TEST_F(ODriveFeedbackTest, ExistingMotionFeedbackAndCommandsRemainAvailable) {
    Get_Encoder_Estimates_msg_t encoder{};
    encoder.Pos_Estimate = 0.25f;
    encoder.Vel_Estimate = -0.5f;
    Get_Torques_msg_t torque{};
    torque.Torque_Target = 1.5f;
    torque.Torque_Estimate = 1.25f;
    received_frames.push_back(frame(encoder));
    received_frames.push_back(frame(torque));
    read();
    EXPECT_DOUBLE_EQ(value("position"), M_PI / 2);
    EXPECT_DOUBLE_EQ(value("velocity"), -M_PI);
    EXPECT_DOUBLE_EQ(value("effort"), 1.5);
    auto commands = hardware_.export_command_interfaces();
    EXPECT_EQ(commands.size(), 6u);
    for (const auto& joint : {"left", "right"}) {
        for (const auto& field : {"position", "velocity", "effort"}) {
            const auto name = std::string(joint) + "/" + field;
            EXPECT_TRUE(std::any_of(commands.begin(), commands.end(), [&](const auto& command) {
                return command.get_name() == name;
            })) << name;
        }
    }
    EXPECT_TRUE(sent_frames.empty());
}

} // namespace

// Test stand-in for odrive_base/src/socket_can.cpp: no CAN bus, every frame
// sent by the hardware interface is recorded in fake_can_sent_frames().
#include "fake_socket_can.hpp"

std::vector<can_frame>& fake_can_sent_frames() {
    static std::vector<can_frame> frames;
    return frames;
}

bool SocketCanIntf::init(const std::string& interface, EpollEventLoop* event_loop, FrameProcessor frame_processor) {
    interface_ = interface;
    event_loop_ = event_loop;
    frame_processor_ = std::move(frame_processor);
    return true;
}

void SocketCanIntf::deinit() {}

bool SocketCanIntf::send_can_frame(const can_frame& frame) {
    fake_can_sent_frames().push_back(frame);
    return true;
}

bool SocketCanIntf::read_nonblocking() {
    return false;
}

void SocketCanIntf::on_socket_event(uint32_t) {}

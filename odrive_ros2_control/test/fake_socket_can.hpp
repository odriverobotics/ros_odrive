#pragma once

#include "socket_can.hpp"

#include <vector>

// Frames sent through the fake SocketCanIntf, oldest first.
std::vector<can_frame>& fake_can_sent_frames();

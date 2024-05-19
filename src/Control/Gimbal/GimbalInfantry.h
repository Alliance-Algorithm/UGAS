#pragma once
/*
Creation Date: 2023/08/02
Latest Update: 2023/08/02
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 步兵云台控制
*/

#include "Core/Tracker/TrackerStruct.h"
#include <cstdint>
#include <rmcs_core/msgs.hpp>

class GimbalInfantry {
public:
    GimbalInfantry() = default;

    void Always(
        TargetInterface*& target_ref, std::chrono::steady_clock::time_point& timestamp_ref,
        rmcs_core::msgs::RoboticColor color, uint8_t robot_id);
};

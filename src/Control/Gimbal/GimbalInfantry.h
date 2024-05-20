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
#include <rmcs_executor/component.hpp>

class GimbalInfantry {
public:
    GimbalInfantry() = default;

    void Always(
        TargetInterface*& target_ref, std::chrono::steady_clock::time_point& timestamp_ref,
        rmcs_executor::Component::InputInterface<rmcs_core::msgs::RoboticColor>& color,
        rmcs_executor::Component::InputInterface<uint8_t>& robot_id,
        std::chrono::milliseconds exposure_time,
        rmcs_executor::Component::InputInterface<bool>& buff_mode, int64_t armor_predict_duration,
        int64_t buff_predict_duration);
};

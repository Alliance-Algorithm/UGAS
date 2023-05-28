#pragma once
/*
Creation Date: 2023/04/01
Latest Update: 2023/04/01
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 基础自瞄策略
- 击打离准星偏角最小的装甲板
*/

#include <cmath>

#include <vector>
#include <optional>

#include "Util/Parameter/Parameters.h"

class Strategy_V1 {
public:
    template <typename TargetType>
    int GetTargetIndex(const std::vector<TargetType>& targets) {
        int i = 0, targetIndex = -1;
        if constexpr (true) {
            // 击打距离最近的
            float distanceMin;
            for (const auto& target : targets) {
                const auto& x = target.position.x(), & y = target.position.y(), & z = target.position.z();
                const auto distance = sqrt(x * x + y * y + z * z);
                if (i == 0 || distanceMin > distance) {
                    targetIndex = i;
                    distanceMin = distance;
                }
                // std::cout << distance << std::endl;
            }
            ++i;
        }
        else {
            // 击打离准星最近的
            float angleMin;
            for (const auto& target : targets) {
                const auto yaw = target.gimbalAttitude.yaw / 180 * static_cast<float>(MathConsts::Pi);
                const auto pitch = target.gimbalAttitude.pitch / 180 * static_cast<float>(MathConsts::Pi);
                const auto x1 = cos(yaw) * cos(pitch), y1 = sin(yaw) * cos(pitch), z1 = sin(pitch);
                const auto& x2 = target.position.x, & y2 = target.position.y, & z2 = target.position.z;
                const auto angle = acos((x1 * x2 + y1 * y2 + z1 * z2) / sqrt(x2 * x2 + y2 * y2 + z2 * z2));
                if (i == 0 || angleMin > angle) {
                    targetIndex = i;
                    angleMin = angle;
                }
                // std::cout << angle * 180 / static_cast<float>(MathConsts::Pi) << std::endl;
            }
            ++i;
        }
        
        return targetIndex;
    }
};
#pragma once
/*
Creation Date: 2023/05/27
Latest Update: 2023/05/27
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 最基础的预测器，不使用陀螺仪，也不做预测，仅提供基础的目标选择。
*/

#include <cmath>

#include <vector>
#include <optional>

#include <opencv2/opencv.hpp>

#include "Core/PnPSolver/ArmorPnPSolverInterface.h"
#include "Util/TimeStamp/TimeStampCounter.h"


class SimplePredictor {
public:
    struct Target : private ArmorPlate3d {
    public:
        explicit Target(const ArmorPlate3d& armorPlate3d) : ArmorPlate3d(armorPlate3d) { }

        [[nodiscard]] auto Predict(float sec) const {
            return position;
        }
    };

    SimplePredictor() = default;
    SimplePredictor(const SimplePredictor&) = delete;
    SimplePredictor(SimplePredictor&&) = delete;

    std::optional<Target> Update(const std::vector<ArmorPlate3d>& armors3d, std::chrono::steady_clock::time_point timeStamp) {
        if (!armors3d.empty()) {
            std::vector<double> distances;
            std::vector<double> angles;
            for (const auto& armor3d : armors3d) {
                const double& x = armor3d.position->x(), & y = armor3d.position->y(), & z = armor3d.position->z();
                double dis = sqrt(x * x + y * y + z * z);
                distances.push_back(dis);
                double angle = x / dis;
                angles.push_back(angle);
            }

            for (const auto& armor3d : armors3d) {
                if (armor3d.id == ArmorID::Hero) {
                    lastTarget = ArmorID::Hero;
                    lastSuccess = timeStamp;
                    break;
                }
            }

            std::vector<std::size_t> sortedAngles;
            for (std::size_t i = 0; i < armors3d.size(); ++i)
                sortedAngles.push_back(i);
            std::sort(sortedAngles.begin(), sortedAngles.end(), [&angles](const int& a, const int& b) -> bool {
                return angles[a] < angles[b];
            });

            if (lastTarget != ArmorID::Unknown) {
                if (timeStamp - lastSuccess < std::chrono::milliseconds(100)) {
                    for (const auto& i : sortedAngles)
                        if (armors3d[i].id == lastTarget)
                            return Target(armors3d[i]);
                    return std::nullopt;
                }
                else lastTarget = ArmorID::Unknown;
            }

            return Target(armors3d[0]);
        }
        else return std::nullopt;
    }

private:
    ArmorID lastTarget = ArmorID::Unknown;
    std::chrono::steady_clock::time_point lastSuccess;
};
#pragma once
/*
Creation Date: 2023/06/09
Latest Update: 2023/06/09
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:

*/

#include <cmath>

#include <vector>
#include <optional>

#include <opencv2/opencv.hpp>

#include "Core/PnPSolver/ArmorPnPSolverInterface.h"
#include "Util/TimeStamp/TimeStampCounter.h"
#include "Util/Parameter/Parameters.h"

class VerticalTracker {
public:

    struct Target {
    public:
        explicit Target(const Eigen::Vector3d& pos, double yaw, double v_yaw) : _pos(pos), _yaw(yaw), _v_yaw(v_yaw) { }

        Eigen::Vector3d Predict(float sec) const {
            return _pos;
        }

        bool Shotable(float sec) const {
            //double yaw_diff = std::fmod(_yaw + _v_yaw * sec, 2 * MathConsts::Pi / 6);

            constexpr double div = 2 * MathConsts::Pi / 3;
            double yaw_diff = std::fmod(_yaw + _v_yaw * sec, div);
            //std::cout << yaw_diff << '\n';
            if (yaw_diff < 0) yaw_diff += div;

            return (yaw_diff < 0.6 || yaw_diff > div - 0.6);
        }

    private:
        Eigen::Vector3d _pos;
        double _yaw, _v_yaw;
    };


    class Tracker {
    public:
        void Update(const ArmorPlate3d& armor3d, std::chrono::steady_clock::time_point timeStamp) {
            Eigen::Vector2d vec1 = { -armor3d.position.x(), -armor3d.position.y() };
            Eigen::Vector2d vec2 = { armor3d.normal.x(), armor3d.normal.y() };
            
            double angle = acos(vec1.dot(vec2) / (vec1.norm() * vec2.norm()));
            if (vec1.x() * vec2.y() - vec1.y() * vec2.x() < 0)
                angle = -angle;

            if (std::fabs(angle) < 0.01) {
                _verticalPos = armor3d.position;
                std::cout << angle << '\n';
            }

            //std::cout << armor3d.normal.x() << ' ' << armor3d.normal.y() << ' ' << armor3d.normal.z() << '\n';
            //std::cout << angle << '\n';

        }

        Target GetTarget() {
            return Target(_verticalPos, 0, 0);
        }

    private:
        Eigen::Vector3d _verticalPos;
    };


    std::optional<Target> Update(const std::vector<ArmorPlate3d>& armors3d, std::chrono::steady_clock::time_point timeStamp) {

        if (!armors3d.empty()) {
            std::vector<double> distances;
            std::vector<double> angles;
            for (const auto& armor3d : armors3d) {
                const double& x = armor3d.position.x(), & y = armor3d.position.y(), & z = armor3d.position.z();
                double dis = sqrt(x * x + y * y + z * z);
                distances.push_back(dis);
                double angle = x / dis;
                angles.push_back(angle);
            }

            for (const auto& armor3d : armors3d) {
                if (armor3d.id == ArmorID::Outpost) {
                    lastTarget = ArmorID::Outpost;
                    lastSuccess = timeStamp;
                    break;
                }
                else if (armor3d.id == ArmorID::Hero) {
                    lastTarget = ArmorID::Hero;
                    lastSuccess = timeStamp;
                    break;
                }
            }

            std::vector<int> sortedAngles;
            for (int i = 0; i < armors3d.size(); ++i)
                sortedAngles.push_back(i);
            std::sort(sortedAngles.begin(), sortedAngles.end(), [&angles](const int& a, const int& b) -> bool {
                return angles[a] < angles[b];
            });

            if (lastTarget != ArmorID::Unknown) {
                if (timeStamp - lastSuccess < std::chrono::milliseconds(500)) {
                    if (lastTarget == ArmorID::Outpost) {
                        for (const auto& i : sortedAngles)
                            if (armors3d[i].id == lastTarget) {
                                tracker.Update(armors3d[i], timeStamp);
                                return tracker.GetTarget();
                            }
                        //tracker.Update(timeStamp);
                        //return tracker.GetTarget();
                        return {};
                    }
                    else {
                        for (const auto& i : sortedAngles)
                            if (armors3d[i].id == lastTarget)
                                return Target(armors3d[i].position, 0, 0);
                    }
                }
                else lastTarget = ArmorID::Unknown;
            }

            return Target(armors3d[0].position, 0, 0);
        }


        /*if (!armors3d.empty()) {
            auto& armor3d = armors3d[0];
            tracker.Update(armor3d, timeStamp);
            return tracker.GetTarget();
        }*/

        return {};

    }


    ArmorID lastTarget = ArmorID::Unknown;
    std::chrono::steady_clock::time_point lastSuccess;

    Tracker tracker;
};

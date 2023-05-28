#pragma once
/*
Creation Date: 2022/11/19
Latest Update: 2023/03/17
Developer(s): 21-THY 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 将图像坐标解算为机器人本体坐标
- 使用陀螺仪四元数进行解算
*/

//#include <cmath>

#include <optional>
#include <vector>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "Core/PnPSolver/ArmorPnPSolverInterface.h"
#include "Core/Identifier/Armor/ArmorStruct.h"
#include "Util/Parameter/Parameters.h"
#include "Control/Gimbal/Gimbal.h"
#include "Control/Serial/HiPNUC.h"


class ArmorPnPSolver {
public:
    ArmorPnPSolver() { }
    ArmorPnPSolver(const ArmorPnPSolver&) = delete;
    ArmorPnPSolver(ArmorPnPSolver&&) = delete;


    std::optional<Eigen::Vector3d> Solve(const ArmorPlate& armor) {
        cv::Mat rvec, tvec;

        auto& objectPoints = armor.is_large_armor ? LargeArmor3f : NormalArmor3f;
        if (cv::solvePnP(objectPoints, armor.points, CameraMatrix, DistCoeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE)) {
            double x = tvec.at<double>(2), y = -tvec.at<double>(0), z = -tvec.at<double>(1);
            return Eigen::Vector3d{ x, y, z };
        }

        return std::nullopt;
    }

    std::vector<ArmorPlate3d> SolveAll(const std::vector<ArmorPlate> armors) {
        std::vector<ArmorPlate3d> armors3d;

        for (const auto& armor : armors) {
            if (auto&& pos = Solve(armor)) {
                armors3d.emplace_back(armor.id, *pos);
            }
        }

        return armors3d;
    }
    
    template <typename TransformerType>
    std::vector<ArmorPlate3d> SolveAll(const std::vector<ArmorPlate> armors, TransformerType transformer) {
        std::vector<ArmorPlate3d> armors3d;

        for (const auto& armor : armors) {
            if (auto&& pos = Solve(armor)) {
                auto tranPos = transformer.Link2Gyro(transformer.CameraLink2GimbalLink(*pos));
                armors3d.emplace_back(armor.id, tranPos);
            }
        }

        return armors3d;
    }
};


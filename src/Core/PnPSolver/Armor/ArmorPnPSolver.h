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
#include <eigen3/Eigen/Dense>

#include "Core/PnPSolver/ArmorPnPSolverInterface.h"
#include "Core/Identifier/Armor/ArmorStruct.h"
#include "Util/Parameter/Parameters.h"
#include "Control/Serial/HiPNUC.h"


class ArmorPnPSolver {
public:
    ArmorPnPSolver() = default;
    ArmorPnPSolver(const ArmorPnPSolver&) = delete;
    ArmorPnPSolver(ArmorPnPSolver&&) = delete;


    static std::optional<std::tuple<CameraLink::Position, CameraLink::Rotation>> Solve(const ArmorPlate& armor) {
        cv::Mat rvec, tvec;

        auto& objectPoints = armor.is_large_armor ? parameters::LargeArmorObjectPoints : parameters::NormalArmorObjectPoints;
        if (cv::solvePnP(objectPoints, armor.points,
                         parameters::CameraMatrix, parameters::CameraDistCoeffs,
                         rvec, tvec, false, cv::SOLVEPNP_IPPE)) {
            if constexpr (debugCanvas.master) {
//                std::vector<cv::Point2d> projected_points;
//                cv::projectPoints(objectPoints, rvec, tvec, parameters::CameraMatrix, parameters::CameraDistCoeffs, projected_points);
//                cv::circle(debugCanvas.master.GetMat(), projected_points[0], 2, COLOR_ORANGE, 2);
//                cv::circle(debugCanvas.master.GetMat(), projected_points[1], 2, COLOR_PINK, 2);
//                cv::circle(debugCanvas.master.GetMat(), projected_points[2], 2, COLOR_PURPLE, 2);
//                cv::circle(debugCanvas.master.GetMat(), projected_points[3], 2, COLOR_RED, 2);
            }

            Eigen::Vector3d position = {tvec.at<double>(2), -tvec.at<double>(0), -tvec.at<double>(1)};
            position = position / 1000.0;
            if (position.norm() > parameters::MaxArmorDistance) return std::nullopt;

            Eigen::Vector3d rvec_eigen = { rvec.at<double>(2), -rvec.at<double>(0), -rvec.at<double>(1) };
            Eigen::Quaterniond rotation = Eigen::Quaterniond{Eigen::AngleAxisd{rvec_eigen.norm(), rvec_eigen.normalized()}};

            return std::tuple<CameraLink::Position, CameraLink::Rotation>{position, rotation};
        }
        else return std::nullopt;
    }

    static std::vector<ArmorPlate3d> SolveAll(const std::vector<ArmorPlate>& armors) {
        std::vector<ArmorPlate3d> armors3d;

        for (const auto& armor : armors) {
            if (auto&& result = Solve(armor)) {
                auto& [position, rotation] = *result;
                armors3d.emplace_back(armor.id, position, rotation);
            }
        }

        return armors3d;
    }
};


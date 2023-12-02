#pragma once
/*
Creation Date: 2023/8/4
Latest Update: 2023/8/4
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
 - 单片幅片（4点）的PnP解算
*/

#include <opencv2/opencv.hpp>

#include "Core/Identifier/Buff/BuffStruct.h"

class BuffPnPSolver {
public:
    BuffPnPSolver() = default;
    BuffPnPSolver(const BuffPnPSolver&) = delete;
    BuffPnPSolver(BuffPnPSolver&&) = delete;

    static std::optional<BuffPlate3d> Solve(const BuffPlate& buff) {
        cv::Mat rvec, tvec;

        if (cv::solvePnP(parameters::BuffObjectPoints, buff.points,
                         parameters::CameraMatrix, parameters::CameraDistCoeffs,
                         rvec, tvec, false, cv::SOLVEPNP_IPPE)) {
            Eigen::Vector3d position = {tvec.at<double>(2), -tvec.at<double>(0), -tvec.at<double>(1)};
            position = position / 1000.0;
            if (position.norm() > parameters::MaxArmorDistance) return std::nullopt;

            Eigen::Vector3d rvec_eigen = { rvec.at<double>(2), -rvec.at<double>(0), -rvec.at<double>(1) };
            Eigen::Quaterniond rotation = Eigen::Quaterniond{Eigen::AngleAxisd{rvec_eigen.norm(), rvec_eigen.normalized()}};

            return BuffPlate3d{CameraLink::Position{position}, CameraLink::Rotation{rotation}};
        }
        return std::nullopt;
    }
};


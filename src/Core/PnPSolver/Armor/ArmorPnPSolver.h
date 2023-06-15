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
#include "Control/Gimbal/Gimbal.h"
#include "Control/Serial/HiPNUC.h"


class ArmorPnPSolver {
public:
    ArmorPnPSolver() { }
    ArmorPnPSolver(const ArmorPnPSolver&) = delete;
    ArmorPnPSolver(ArmorPnPSolver&&) = delete;


    std::optional<std::tuple<Eigen::Vector3d, Eigen::Vector3d>> Solve(const ArmorPlate& armor) {
        cv::Mat rvec, tvec;

        //std::vector<cv::Point2f> armorPoints = { {0, 0}, {200 };
        /*for (const auto& point : armor.points) {
            std::cout << point << ' ';
        }
        std::cout << '\n';*/

        auto& objectPoints = armor.is_large_armor ? LargeArmor3f : NormalArmor3f;
        if (cv::solvePnP(objectPoints, armor.points, CameraMatrix, DistCoeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE)) {
            std::tuple<Eigen::Vector3d, Eigen::Vector3d> result;
            auto& [pos, normal] = result;

            //cv::Mat RotateMatrix;
            //cv::Rodrigues(rvec, RotateMatrix); // RotateMatrix is 3x3

            //cv::Mat TransformMatrix = cv::Mat::eye(4, 4, CV_64F);

            //TransformMatrix(cv::Range(0, 3), cv::Range(0, 3)) = RotateMatrix;
            //std::cout << tvec.type() << '\n';
            ////TransformMatrix.at<(3, 0)
            //TransformMatrix(cv::Range(0, 3), cv::Range(3, 4)) = tvec;

            //std::cout << TransformMatrix;

            double x = tvec.at<double>(2), y = -tvec.at<double>(0), z = -tvec.at<double>(1);
            pos = { x, y, z };

            Eigen::Vector3d rvecEigen = { rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2) };
            Eigen::Matrix3d R = Eigen::AngleAxisd(rvecEigen.norm(), rvecEigen.normalized()).toRotationMatrix();
            {
                normal = R * Eigen::Vector3d{ 0, 0, 1 };
                if (normal.z() > 0)
                    normal = R * Eigen::Vector3d{ 0, 0, -1 };
                normal = { normal.z(), -normal.x(), -normal.y() };
            }

            //std::cout << normal.x() << ' ' << normal.y() << ' ' << normal.z() << '\n';

            return result;

            /*Eigen::Matrix3d R = Eigen::AngleAxisd(rvecEigen.norm(), rvecEigen.normalized()).toRotationMatrix();
            Eigen::Vector3d T = { tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2) };
            Eigen::Matrix4d Trans;
            Trans.setIdentity();
            Trans.block<3, 3>(0, 0) = R;
            Trans.block<3, 1>(0, 3) = T;

            Eigen::Vector4d vec = { 0, 1, 0, 0 };
            vec = Trans * vec;
            
            std::cout << vec[0] << ' ' << vec[1] << ' ' << vec[2] << '\n';*/
            //Eigen::Quaterniond coordRotate = { 0.5, -0.5, 0.5, -0.5 };
            //Eigen::Quaterniond q = Eigen::Quaterniond(Eigen::AngleAxisd(rvecEigen.norm(), rvecEigen.normalized()));
            /*Eigen::Matrix3d rotate = Eigen::AngleAxisd(rvecEigen.norm(), rvecEigen.normalized()).toRotationMatrix();
            Eigen::Matrix3d ros2cv, cv2ros;
            ros2cv << 0, -1, 0, 0, 0, -1, 1, 0, 0;
            cv2ros << 0, 0, 1, -1, 0, 0, 0, -1, 0;
            rotate = cv2ros * rotate * ros2cv;*/

            //Eigen::Quaterniond vec = { 0, 0, 0, 1 };
            //Eigen::Vector3d vec = rotate * ros2cv * Eigen::Vector3d{ 1, 0, 0 };

            //std::this_thread::sleep_for(std::chrono::milliseconds(100));
            //std::cout << tvec.at<double>(0) << ' ' << tvec.at<double>(1) << ' ' << tvec.at<double>(2) << '\n';
            //std::cout << vec[0] << ' ' << vec[1] << ' ' << vec[2] << '\n';
            //std::cout << rvecEigen[0] << ' ' << rvecEigen[1] << ' ' << rvecEigen[2] << '\n';
        }

        return std::nullopt;
    }

    std::vector<ArmorPlate3d> SolveAll(const std::vector<ArmorPlate> armors) {
        std::vector<ArmorPlate3d> armors3d;

        for (const auto& armor : armors) {
            if (auto&& result = Solve(armor)) {
                auto& [pos, normal] = *result;
                armors3d.emplace_back(armor.id, pos, normal);
            }
        }

        return armors3d;
    }
    
    template <typename TransformerType>
    std::vector<ArmorPlate3d> SolveAll(const std::vector<ArmorPlate> armors, TransformerType transformer) {
        std::vector<ArmorPlate3d> armors3d;

        //std::cout << armors.size();
        for (const auto& armor : armors) {
            if (auto&& result = Solve(armor)) {
                auto& [pos, normal] = *result;
                auto tranPos = transformer.Link2Gyro(transformer.CameraLink2GimbalLink(pos));
                auto tranNormal = transformer.Link2Gyro(normal);
                armors3d.emplace_back(armor.id, tranPos, tranNormal);
            }
        }

        return armors3d;
    }
};


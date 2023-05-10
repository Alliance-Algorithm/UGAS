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

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "Core/PnPSolver/ArmorPnPSolverInterface.h"
#include "Core/Identifier/Armor/ArmorStruct.h"
#include "Util/Parameter/Parameters.h"
#include "Control/Gimbal/Gimbal.h"

#include "Control/Serial/HiPNUC.h"


template <typename IMUType>
class ArmorPnPSolver_V2 {
public:
	ArmorPnPSolver_V2(IMUType& imu) : _imu(imu) { }
	ArmorPnPSolver_V2(const ArmorPnPSolver_V2&) = delete;
	ArmorPnPSolver_V2(ArmorPnPSolver_V2&&) = delete;


	std::optional<ArmorPlate3d> Solve(const ArmorPlate& armor) {
		cv::Mat rvec, tvec;

		auto& objectPoints = armor.is_large_armor ? LargeArmor3f : NormalArmor3f;
		if (cv::solvePnP(objectPoints, armor.points, CameraMatrix, DistCoeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE)) {

			float x = tvec.at<double>(2), y = tvec.at<double>(0), z = -tvec.at<double>(1);

			auto pos = _imu.Transform(Eigen::Vector3f{ x, y, z });

			//std::cout << rvec << '\n';

			//std::cout << x << ' ' << y << ' ' << z << '\n';
			//std::cout << x << ' ' << pos.x() << "      " << y << ' ' << pos.y() << "      " << z << ' ' << pos.z() << '\n';
			//auto&& x2 = sqrt(x * x + z * z);
			//auto&& pitch = atan2(z, x) * 180 / MathConsts::Pi;

			//std::cout << GimbalAttitude(0, pitch) << ' ' << cv::Point3f(x2, y, 0) << std::endl;
			return ArmorPlate3d(armor.id, GimbalAttitude(0, 0), {pos.x(), pos.y(), pos.z()});

			// TODO: 欧拉角解算还没写
		}

		return std::nullopt;
	}

private:
	const IMUType& _imu;
};


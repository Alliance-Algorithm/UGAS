#pragma once
/*
Creation Date: 2022/11/19
Latest Update: 2023/03/17
Developer(s): 21-THY 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 将图像坐标解算为机器人本体坐标
- 不使用陀螺仪，假定相机和装甲板在同一水平线上进行计算
*/

#include <cmath>

#include <optional>

#include <opencv2/opencv.hpp>

#include "Core/PnPSolver/ArmorPnPSolverInterface.h"
#include "Core/Identifier/Armor/ArmorStruct.h"
#include "Util/Parameter/Parameters.h"
#include "Control/Gimbal/Gimbal.h"


class ArmorPnPSolver_V1 : public ArmorPnPSolverInterface {
private:

public:
	ArmorPnPSolver_V1() {
		UpdateGimbalAttitude(.0, .0);
	}
	ArmorPnPSolver_V1(const ArmorPnPSolver_V1&) = delete;
	ArmorPnPSolver_V1(ArmorPnPSolver_V1&&) = delete;

	void UpdateGimbalAttitude(double pitch, double yaw) override { }

	std::optional<ArmorPlate3d> Solve(const ArmorPlate& armor, bool isLargeArmor) override {
		cv::Mat rvec, tvec;

		auto& objectPoints = isLargeArmor ? LargeArmor3f : NormalArmor3f;
		if (cv::solvePnP(objectPoints, armor.points,
			CameraMatrix, DistCoeffs,
			rvec, tvec, false, cv::SOLVEPNP_IPPE)) {

			auto &x = tvec.at<double>(2), &y = tvec.at<double>(0), &z = tvec.at<double>(1);
			auto&& x2 = sqrt(x * x + z * z);
			auto&& pitch = atan2(z, x) * 180 / MathConsts::Pi;

			//std::cout << GimbalAttitude(0, pitch) << ' ' << cv::Point3f(x2, y, 0) << std::endl;
			return ArmorPlate3d(armor.id, GimbalAttitude(0, pitch), cv::Point3f(x2, y, 0));

			// TODO: 欧拉角解算还没写
		}

		return {};
	}
};


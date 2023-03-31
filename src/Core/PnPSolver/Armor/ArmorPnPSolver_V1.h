#pragma once
/*
Creation Date: 2022/11/19
Latest Update: 2023/03/17
Developer(s): 21-THY 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 将图像坐标解算为世界坐标
*/

#include "Core/PnPSolver/ArmorPnPSolverInterface.h"
#include "Util/Parameter/Parameters.h"

class ArmorPnPSolver_V1 : public ArmorPnPSolverInterface {
private:
	cv::Mat _CameraMatrix;

	// 云台姿态
	double _pitch, _yaw;
	// _transMat：相机坐标转世界坐标 _revertMat: 世界坐标转相机坐标
	cv::Mat _transMat, _revertMat;

public:
	ArmorPnPSolver_V1() {
		_CameraMatrix = (cv::Mat_<float>(3, 3) <<
			CameraMatrixData[0][0], CameraMatrixData[0][1], CameraMatrixData[0][2],
			CameraMatrixData[1][0], CameraMatrixData[1][1], CameraMatrixData[1][2],
			CameraMatrixData[2][0], CameraMatrixData[2][1], CameraMatrixData[2][2]);
		UpdateGimbalAttitude(.0, .0);
	}
	ArmorPnPSolver_V1(const ArmorPnPSolver_V1&) = delete;
	ArmorPnPSolver_V1(ArmorPnPSolver_V1&&) = delete;

	void UpdateGimbalAttitude(double pitch, double yaw) override {
		_pitch = pitch, _yaw = yaw;
		_transMat = (cv::Mat_<float>(3, 3) <<
			-sin(_yaw), sin(_pitch) * cos(_yaw), cos(_pitch) * cos(_yaw),
			cos(_yaw), sin(_pitch) * sin(_yaw), cos(_pitch) * sin(_yaw),
			0, -cos(_pitch), sin(_pitch));
		cv::transpose(_transMat, _revertMat);
	}

	cv::Point3f Solve(const ArmorPlate& armor, bool isLargeArmor) override {
		cv::Point3f result(0, 0, 0);
		cv::Mat rvec, tvec;

		auto& objectPoints = isLargeArmor ? LargeArmor3f : NormalArmor3f;
		if (cv::solvePnP(objectPoints, armor.points,
			CameraMatrix, DistCoeffs,
			rvec, tvec, false, cv::SOLVEPNP_IPPE)) {

			cv::Mat position = _transMat * (cv::Mat_<float>(3, 1) <<
				tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

			result.x = position.at<float>(0);
			result.y = position.at<float>(1);
			result.z = position.at<float>(2);

			std::cout << result << std::endl;

			// cv::Rodrigues(rvec, rvec);
			// 这还有欧拉角解算还没写，也没留接口，要用再说
		}

		return result;
	}

	//[[deprecated("暂时不使用这个函数")]]
	cv::Point2f RevertPnP(const cv::Point3f position) {
		cv::Mat pos = _CameraMatrix * (_revertMat *
			(cv::Mat_<float>(3, 1) << position.x, position.y, position.z));
		//LOG(INFO) << CameraMatrix << '\n' << _CameraMatrix << '\n' << _revertMat << '\n' \
		//	<< (cv::Mat_<float>(3, 1) << position.x, position.y, position.z) << '\n' << pos << '\n';
		pos /= pos.at<float>(2);
		return cv::Point2f(pos.at<float>(0), pos.at<float>(1));
	}
};


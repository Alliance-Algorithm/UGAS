#include "PnP.h"
#include "Parameters.h"

PnP PnPsolver;

void PnP::GetTransMat() {
	_pitch = _com.Get().pitchA * PI / 180;
	_roll  = _com.Get().rollA  * PI / 180;
	_yaw   = _com.Get().yawA   * PI / 180;
	// 没考虑roll轴
	_transMat = (cv::Mat_<float>(3, 3) <<
		-sin(_yaw), sin(_pitch)* cos(_yaw), cos(_pitch)* cos(_yaw),
		cos(_yaw), sin(_pitch)* sin(_yaw), cos(_pitch)* sin(_yaw),
		0, -cos(_pitch), sin(_pitch));
	cv::transpose(_transMat, _revertMat);
}

PnP::PnP() :_com(), 
	_pitch(.0), _roll(.0), _yaw(.0), _transMat(), _revertMat() {}

PnP::PnP(serial::GimbalSerialHandle com) :_com(com),
	_pitch(.0), _roll(.0), _yaw(.0), _transMat(), _revertMat() {}

cv::Point3f PnP::SolvePnP(const ArmorPlate& armor) {
	cv::Point3f result;
	cv::Mat rvec, tvec;

	if (isLargeArmor[armor.id]) {
		cv::solvePnP(LargeArmor3f, armor.points,
			CameraMatrix, DistCoeffs,
			rvec, tvec, false, cv::SOLVEPNP_AP3P);
	}
	else {
		cv::solvePnP(NormalArmor3f, armor.points,
			CameraMatrix, DistCoeffs,
			rvec, tvec, false, cv::SOLVEPNP_AP3P);
	}

	GetTransMat();
	cv::Mat position = tvec * _transMat;
	result.x = position.at<double>(0);
	result.y = position.at<double>(1);
	result.z = position.at<double>(2);

	cv::Rodrigues(rvec, rvec);
	// 这还有欧拉角解算还没写，也没留接口，要用再说

	return result;
}

cv::Point2f PnP::RevertPnP(const cv::Point3f position) {
	cv::Mat pos =
		(cv::Mat_<float>(3, 1) << position.x, position.y, position.z)
		* _revertMat * CameraMatrix;
	pos /= pos.at<float>(2);
	return cv::Point2f(pos.at<float>(0), pos.at<float>(1));
}

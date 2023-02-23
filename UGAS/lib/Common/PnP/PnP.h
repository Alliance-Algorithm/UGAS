#pragma once
/*
Creation Date: 2022/11/19
Latest Update: 2022/11/19
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 一个典型的单例类
- 负责解算空间与图像坐标的关系
*/
#include <GimbalSerial/GimbalSerialHandle.h>

class PnP {
private:
	cv::Mat _CameraMatrix;

	double _pitch, _roll, _yaw; // pose
	cv::Mat _transMat, _revertMat;
public:
	PnP();

	void GetTransMat();
	cv::Point3f SolvePnP(const ArmorPlate& armor);
	cv::Point2f RevertPnP(const cv::Point3f position);

	friend class UGAS;
};

extern PnP PnPsolver;

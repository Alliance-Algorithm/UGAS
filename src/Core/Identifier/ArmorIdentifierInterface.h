#pragma once
/*
Creation Date: 2022/10/13
Latest Update: 2022/10/13
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 提供装甲识别类的接口
*/

#include <vector>

#include <opencv2/opencv.hpp>

#include "Core/Identifier/Armor/ArmorStruct.h"
#include "Util/Debug/Log.h"

//ToDo: 把这里处理掉
inline double P2PDis(const cv::Point2f& a, const cv::Point2f& b) {
	cv::Point2f tmp = b - a;
	return sqrt(tmp.x * tmp.x + tmp.y * tmp.y);
}

inline double P2PDis(const cv::Point3f& a, const cv::Point3f& b) {
	cv::Point3f tmp = b - a;
	return sqrt(tmp.x * tmp.x + tmp.y * tmp.y + tmp.z * tmp.z);
}

inline double malposition(const LightBar& LBl, const LightBar& LBr) {
	cv::Point2f axis = (LBl.top - LBl.bottom + LBr.top - LBr.bottom) / 2;
	cv::Point2f dis = (LBl.top + LBl.bottom - LBr.top - LBr.bottom) / 2;
	return fabs(axis.dot(dis) / axis.cross(dis));
}

class ArmorIdentifierInterface {
public:
	virtual ~ArmorIdentifierInterface() = default;

	virtual std::vector<ArmorPlate> Identify(const cv::Mat& imgThre, const cv::Mat& imgGray) = 0;
};

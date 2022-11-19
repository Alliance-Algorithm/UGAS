#pragma once
/*
Creation Date: 2022/10/19
Latest Update: 2022/10/19
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 提供存储完整目标信息的封装
*/
#include <opencv2/opencv.hpp>
#include "Common/DebugTools/DebugHeader.h"

class TargetSolution; // 友元函数的预声明

class Target {
private:
	// 考虑融合TimeStamp到Target(明天再改)
	cv::Vec3f _position, _speed;
public:
	Target(cv::Vec3f position = cv::Vec3f(), cv::Vec3f speed = cv::Vec3f()) :
		_position(position), _speed(speed) {}

	cv::Vec3f Predict(int milliSec) const {
		return _position + _speed * milliSec;
	}

	friend class TargetSolution;
};

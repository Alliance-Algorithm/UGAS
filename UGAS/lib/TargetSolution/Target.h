#pragma once
/*
Creation Date: 2022/10/13
Latest Update: 2022/10/13
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 提供存储完整目标信息的封装
*/
#include <opencv.hpp>
#include "TargetSolution.h"

class Target {
private:
	cv::Vec3f _position, _speed;
public:
	Target(cv::Vec3f position, cv::Vec3f speed = cv::Vec3f()) :
		_position(position), _speed(speed) {}

	cv::Vec3f Predict(int milliSec) const {
		return _position + _speed * milliSec;
	}

	friend class TargetSolution;
};

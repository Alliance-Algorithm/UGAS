#pragma once
/*
Creation Date: 2022/11/18
Latest Update: 2022/11/18
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 定义一个描述机器人及其装甲板运动信息的数据模型
*/
#include <opencv2/opencv.hpp>
#include "Common/UniversalStruct.h"

extern enum RotateDirc;

class Robot {
private:
	TimeStamp _latestUpdate;
	double _pitch, _roll, _yaw;
	cv::Vec3f _robotCenter;
	cv::Vec3f _movingSpeed;

	ArmorPlate _armor;

	RotateDirc _rotate;
	double _rotateSpeed; // omiga(w) : rad/s
public:
	Robot();

	void Update(TimeStamp tp, ArmorPlate armor);
	cv::Point3f Predict(int millisec);
};

extern Robot robots[10];

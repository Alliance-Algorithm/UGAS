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
	cv::Point3f _robotCenter;
	cv::Vec3f _movingSpeed;

	ArmorPlate _armor;
	cv::Point3f _armorCenter;

	TimeStamp _rotationLatestUpdate;
	RotateDirc _rotate;
	double _rotateSpeed; // omiga(w) : rad/s
public:
	Robot();

	void Update(TimeStamp ImgTime, const ArmorPlate& armor);
	cv::Point3f Predict(int millisec) const;
};

extern Robot robots[10];

#pragma once
/*
Creation Date: 2022/11/18
Latest Update: 2022/11/18
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- ����һ�����������˼���װ�װ��˶���Ϣ������ģ��
*/
#include <opencv2/opencv.hpp>
#include "Common/TimeStamp/TimeStampCounter.h"

enum RotateDirc;

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

	double _possibility;
public:
	Robot();

	void Update(TimeStamp ImgTime, const ArmorPlate& armor);
	double GetPossibility();
	cv::Point3f Predict(int millisec) const;
};

extern Robot robots[10];
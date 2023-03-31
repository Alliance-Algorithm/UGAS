#pragma once
/*
Creation Date: 2023/03/17
Latest Update: 2023/03/17
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 装甲板PnP解算的接口类
- 将图像坐标解算为世界坐标
*/

#include <opencv2/opencv.hpp>

#include "Core/Identifier/Armor/ArmorStruct.h"

class ArmorPnPSolverInterface {
public:
	virtual ~ArmorPnPSolverInterface() = default;

	virtual void UpdateGimbalAttitude(double pitch, double yaw) = 0;
	// 图像坐标解算为机器人本体坐标
	virtual cv::Point3f Solve(const ArmorPlate& armor, bool isLargeArmor) = 0;
};
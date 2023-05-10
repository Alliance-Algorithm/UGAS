#pragma once
/*
Creation Date: 2023/03/17
Latest Update: 2023/03/17
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 装甲板PnP解算的接口类
- 将图像坐标解算为机器人本体坐标
*/


#include <optional>

#include <opencv2/opencv.hpp>

#include "Control/Gimbal/Gimbal.h"
#include "Core/Identifier/Armor/ArmorStruct.h"


/*
  position: 机器人本体坐标
  坐标系原点与机器人本体中心点重合
  坐标系z轴竖直向上，xy轴仅与机器人启动时的云台指向有关，云台旋转不影响xyz轴
*/
struct ArmorPlate3d {
	ArmorID id;
	GimbalAttitude gimbalAttitude;
	cv::Point3f position;
	ArmorPlate3d(ArmorID _id, const GimbalAttitude& _gimbalAttitude, const cv::Point3f& _position)
		: id(_id), gimbalAttitude(_gimbalAttitude), position(_position) { }
};


class ArmorPnPSolverInterface {
public:
	virtual ~ArmorPnPSolverInterface() = default;

	virtual void UpdateGimbalAttitude(double pitch, double yaw) = 0;
	// 图像坐标解算为机器人本体坐标
	virtual std::optional<ArmorPlate3d> Solve(const ArmorPlate& armor, bool isLargeArmor) = 0;
};
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
#include <Eigen/Dense>

#include "Control/Gimbal/Gimbal.h"
#include "Core/Identifier/Armor/ArmorStruct.h"

struct ArmorPlate3d {
	ArmorID id;
	Eigen::Vector3d position;
	ArmorPlate3d(ArmorID _id, const Eigen::Vector3d& _position) : id(_id), position(_position) { }
};

class ArmorPnPSolverInterface {
public:
	virtual ~ArmorPnPSolverInterface() = default;

	virtual void UpdateGimbalAttitude(double pitch, double yaw) = 0;
	// 图像坐标解算为机器人本体坐标
	virtual std::optional<ArmorPlate3d> Solve(const ArmorPlate& armor, bool isLargeArmor) = 0;
};
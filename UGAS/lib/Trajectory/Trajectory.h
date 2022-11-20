#pragma once
/*
Creation Date: 2022/11/19
Latest Update: 2022/11/19
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 定义弹道解算类的统一接口
*/
#include "GimbalSerial/GimbalSerialHandle.h"
#include "Common/Robot/Robot.h"
#include "Common/PnP/PnP.h"
#include "Common/DebugTools/DebugHeader.h"

class Trajectory {
public:
	virtual void GetShotAngle(const int targetID, TimeStamp ImgTime, double& yaw, double& pitch) = 0;
};

#pragma once
/*
Creation Date: 2022/10/19
Latest Update: 2022/10/19
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 提供一个暂时充数的实现，就叫 TrackingStrategy Ver 1.0
*/
#include "../TrackingStrategy.h"

class TrackingStrategy_V1 : public TrackingStrategy {
public:
	TrackingStrategy_V1(serial::GimbalSerial& com) :
		TrackingStrategy(com) {}

	bool GetTarget(const std::vector<Target>& targets, Target& result) {
		// 不能说是极其敷衍吧，只能说是滥竽充数
		if (targets.empty()) return false;
		result = targets[0];
		return true;
	}
};

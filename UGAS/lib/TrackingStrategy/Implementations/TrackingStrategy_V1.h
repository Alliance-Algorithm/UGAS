#pragma once
/*
Creation Date: 2022/11/19
Latest Update: 2022/11/19
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 提供一个暂时充数的实现，就叫 TrackingStrategy Ver 1.0
*/
#include "../TrackingStrategy.h"

class TrackingStrategy_V1 : public TrackingStrategy {
public:
	int GetTargetID() {
		// 不能说是极其敷衍吧，只能说是滥竽充数
		return 0;
	}
};

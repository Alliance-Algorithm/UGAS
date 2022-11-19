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
		int res = 0;
		double maxPossibility = .0;
		for (int i = 1; i < 10; ++i) {
			double possibility = robots[i].GetPossibility();
			if (possibility > maxPossibility) {
				res = i;
				maxPossibility = possibility;
			}
		}
		return res;
	}
};

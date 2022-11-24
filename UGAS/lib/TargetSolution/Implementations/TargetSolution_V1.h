#pragma once
/*
Creation Date: 2022/10/13
Latest Update: 2022/10/13
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 提供一个暂时充数的实现，就叫 TargetSolution Ver 1.0
*/
#include "../TargetSolution.h"

class TargetSolution_V1 : public TargetSolution {
private:
	static inline bool isValidId(int id);
public:
	void Solve(TimeStamp ImgTime, std::vector<ArmorPlate>& armors);
};

#pragma once
/*
Creation Date: 2022/10/11
Latest Update: 2022/10/13
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 提供数字识别类的接口
*/
#include "../../Common/UniversalStruct.h"

class NumberIdentifier {
public:
	virtual void init(void*) = 0;
	virtual short Identify(const Img& img, const ArmorPlate& region) = 0;
};

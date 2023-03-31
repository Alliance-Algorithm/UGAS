#pragma once
/*
Creation Date: 2022/10/13
Latest Update: 2022/10/13
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 提供数字识别类的接口
*/

#include <opencv2/opencv.hpp>

#include "Core/Identifier/Armor/ArmorStruct.h"

class NumberIdentifierInterface {
public:
	virtual ~NumberIdentifierInterface() = default;

	virtual short Identify(const cv::Mat& imgGray, const ArmorPlate& region) = 0;
};

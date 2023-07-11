#pragma once
/*
Creation Date: 2023/5/22
Latest Update: 2023/5/22
Developer(s): 21-WZY 21-WTR
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- interface for identify power rune
*/

#include <opencv2/opencv.hpp>

#include "Core/Identifier/Buff/BuffStruct.h"
#include "Util/Debug/Log.h"


class BuffIdentifierInterface {
public:
	virtual ~BuffIdentifierInterface() = default;

	virtual Buff5PointIdentifyData Identify(const cv::Mat& img, const TimeStamp& timeStamp) = 0;
};

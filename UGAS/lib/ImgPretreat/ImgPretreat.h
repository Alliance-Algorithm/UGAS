#pragma once
/*
Creation Date: 2022/10/13
Latest Update: 2022/10/13
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 提供图片预处理类的接口
*/
#include "GimbalSerial/GimbalSerialHandle.h"
#include "Common/DebugTools/DebugHeader.h"

class ImgPretreat {
protected:
	serial::GimbalSerialHandle _com;
public:
	ImgPretreat(const serial::GimbalSerialHandle& com) :_com(com) {}

	virtual void GetPretreated(const cv::Mat& img, cv::Mat& imgThre, cv::Mat& imgGray) = 0;
};

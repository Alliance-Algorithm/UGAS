#pragma once
/*
Creation Date: 2022/10/13
Latest Update: 2022/10/13
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 提供一个暂时充数的实现，就叫 ImgPretreat Ver 1.0
*/
#include "../ImgPretreat.h"

class ImgPretreat_V1 : public ImgPretreat {
public:
	void GetPretreated(const cv::Mat& img, cv::Mat& imgThre, cv::Mat& imgGray);
};

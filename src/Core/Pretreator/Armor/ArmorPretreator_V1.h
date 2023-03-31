#pragma once
/*
Creation Date: 2022/10/13
Latest Update: 2022/10/13
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 提供一个暂时充数的实现，就叫 ImgPretreat Ver 1.0
*/

#include "Core/Pretreator/PretreatorInterface.h"
#include "Util/Debug/Log.h"
#include "Util/Parameter/Parameters.h"

class ArmorPretreator_V1 : public PretreatorInterface {
public:
	std::tuple<cv::Mat, cv::Mat> GetPretreated(const cv::Mat& img) const override;
};

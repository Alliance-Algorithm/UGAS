#pragma once
/*
Creation Date: 2022/10/23
Latest Update: 2022/10/23
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 基于HSL的思想和opencv源码进行快速带选色的三值化
- 剪去不需要的计算大幅加快了性能
*/
#include "Core/Pretreator/PretreatorInterface.h"
#include "Core/Identifier/Color/ColorIdentifier_V1.h"

class ArmorPretreator_V2 : public PretreatorInterface {
public:
	ArmorPretreator_V2();
	std::tuple<cv::Mat, cv::Mat> GetPretreated(const cv::Mat& img) const override;

private:
	ColorIdentifier_V1 _colorIdentifier;

    inline void LoopPixel(const uchar* src, uchar* dst, int n) const;
	inline void Threshold(const cv::Mat& src, cv::Mat& dst) const;
};

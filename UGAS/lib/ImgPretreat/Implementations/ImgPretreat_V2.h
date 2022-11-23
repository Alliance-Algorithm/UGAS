#pragma once
/*
Creation Date: 2022/10/23
Latest Update: 2022/10/23
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 基于HSL的思想进行暴力二值化
*/
#include "../ImgPretreat.h"

class ImgPretreat_V2 : public ImgPretreat {
private:
    inline void LoopPixel(const uchar* src, uchar* dst, int n) const;
	inline void Threshold(const cv::Mat& src, cv::Mat& dst) const;
public:
	using ImgPretreat::ImgPretreat;
	void GetPretreated(const cv::Mat& img, cv::Mat& imgThre, cv::Mat& imgGray);
};

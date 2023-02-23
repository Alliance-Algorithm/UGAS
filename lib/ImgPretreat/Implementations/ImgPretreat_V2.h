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
#include "../ImgPretreat.h"

class ImgPretreat_V2 : public ImgPretreat {
private:
    inline void LoopPixel(const uchar* src, uchar* dst, int n) const;
	inline void Threshold(const cv::Mat& src, cv::Mat& dst) const;
public:
	using ImgPretreat::ImgPretreat;
	void GetPretreated(const cv::Mat& img, cv::Mat& imgThre, cv::Mat& imgGray);
};

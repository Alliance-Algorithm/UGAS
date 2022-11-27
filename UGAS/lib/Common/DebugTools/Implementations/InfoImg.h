#pragma once
/*
Creation Date: 2022/11/20
Latest Update: 2022/11/20
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 用于管理、显示调试图像
*/
#include <opencv2/opencv.hpp>
#include <DebugSettings.h>
#include <Common/Color.h>

class InfoImg :public cv::Mat {
	cv::Rect _ROI;
public:
	void Load(const Mat& img, const cv::Rect& ROI) {
#if DEBUG_IMG == 1
		static_cast<cv::Mat&>(*this) = img.clone();
		cv::rectangle(*this, _ROI = ROI, COLOR_YELLOW);
#endif
	}

	void Show() {
#if DEBUG_IMG == 1
		cv::imshow("InfoImg", *this);
		cv::waitKey(1);
#endif
	}

	inline Mat ROI() { return this->operator()(_ROI); }
};

extern InfoImg debugImg;

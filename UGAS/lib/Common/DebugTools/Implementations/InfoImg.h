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

class InfoImg :public cv::Mat {
public:
	void Load(const Mat& img) {
		static_cast<cv::Mat&>(*this) = img.clone();
	}
	void Show() {
		cv::imshow("InfoImg", *this);
		cv::waitKey(1);
	}
};

extern InfoImg debugImg;

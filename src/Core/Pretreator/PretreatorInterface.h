#pragma once
/*
Creation Date: 2022/10/13
Latest Update: 2022/10/13
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 提供图片预处理类的接口
*/

#include <tuple>

#include <opencv2/opencv.hpp>

class PretreatorInterface {
public:
	virtual ~PretreatorInterface() = default;

	virtual std::tuple<cv::Mat, cv::Mat> GetPretreated(const cv::Mat& img) const = 0;
};
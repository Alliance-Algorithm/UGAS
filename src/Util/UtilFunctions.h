#pragma once
/*
Creation Date: 2022/10/13
Latest Update: 2023/03/31
Developer(s): 21-THY 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 声明并实现一些通用计算辅助函数
*/

#include <cmath>

#include <opencv2/opencv.hpp>

inline float P2PDis(const cv::Point2f& a, const cv::Point2f& b) {
    cv::Point2f tmp = b - a;
    return sqrt(tmp.x * tmp.x + tmp.y * tmp.y);
}

inline float P2PDis(const cv::Point3f& a, const cv::Point3f& b) {
    cv::Point3f tmp = b - a;
    return sqrt(tmp.x * tmp.x + tmp.y * tmp.y + tmp.z * tmp.z);
}

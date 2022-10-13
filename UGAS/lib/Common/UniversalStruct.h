#pragma once
/*
Creation Date: 2022/10/11
Latest Update: 2022/10/11
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 声明并定义所有通用结构体
*/
#include <opencv.hpp>

enum Team { Red = 1, Blue = 2 };

typedef unsigned long long TimeStamp;

class MatWithTimeStamp :public cv::Mat {
public:
	TimeStamp timeStamp;
};
typedef MatWithTimeStamp Img;

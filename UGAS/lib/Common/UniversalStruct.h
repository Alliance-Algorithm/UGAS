#pragma once
/*
Creation Date: 2022/10/11
Latest Update: 2022/10/13
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 声明并定义所有通用结构体
*/
#include <opencv2/opencv.hpp>
#include "DebugTools/DebugHeader.h"

enum Team { Red = 1, Blue = 2 };

typedef unsigned long long TimeStamp;

class MatWithTimeStamp :public cv::Mat {
public:
	TimeStamp timeStamp;

	MatWithTimeStamp() :timeStamp(0) {}
	MatWithTimeStamp(const Mat& img) :Mat(img), timeStamp(0) {}
};
typedef MatWithTimeStamp Img;

struct LightBar {
	cv::Point2f top, bottom;
	float angle;

	// 这个自动计算Angle的实现要和识别的地方表示方式同步
	// 看看什么时候搞个统一的标准还是怎么，先不实现这个构造
	LightBar(cv::Point2f _top, cv::Point2f _bottom);
	LightBar(cv::Point2f _top, cv::Point2f _bottom, float angle) :
		top(_top), bottom(_bottom), angle(angle) {}
};

class ArmorPlate {
public:
	std::vector<cv::Point2f> points;
	short id;

	ArmorPlate() :id(0) {} // 用来支持数组
	ArmorPlate(const LightBar& left, const LightBar& right, short _id = 0):
		id(_id) { Set(left, right, _id); }

	// U字型（和PNP参数表示顺序同步（老代码传承的顺序））
	void Set(const LightBar& left, const LightBar& right, short _id = 0) {
		points.push_back(left.top); points.push_back(left.bottom);
		points.push_back(right.bottom); points.push_back(right.top);
		if (_id) id = _id;
	}
	cv::Point2f center() const {
		if (points.size() != 4)
			throw_with_trace(std::runtime_error, "Invalid ArmorPlate object");
		return (points[0] + points[1] + points[2] + points[3]) / 4;
	}
};

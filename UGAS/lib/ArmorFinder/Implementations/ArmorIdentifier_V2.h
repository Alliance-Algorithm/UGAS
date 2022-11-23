#pragma once
/*
Creation Date: 2022/10/13
Latest Update: 2022/10/13
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 使用DFS和并查集实现的灯条查找算法
- 滥竽充数的灯条匹配
*/

#include "../ArmorIdentifier.h"
#include "FastStruct/FastStruct.h"


class ArmorIdentifier_V2 : public ArmorIdentifier {
private:
	std::queue<cv::Point> _dfsQueue;
	FastDisjointSet<cv::Point> _disjointSet;
	FastArray2d<FastDisjointSet<cv::Point>::Node*> _floodMap;
	std::vector<LightBar> _lightBarList;

	std::vector<std::vector<cv::Point>> _floodFindAreas(const cv::Mat& grayImg, int area_val, int flood_val, bool border_engulfment = false);
	void _floodPixel(const cv::Mat& grayImg, int flood_val, FastDisjointSet<cv::Point>::Node* source, int fy, int fx);
	bool _solveToLightbar(const std::vector<cv::Point>& area);
	void _matchArmorPlates(std::vector<ArmorPlate>& result);

public:
	using ArmorIdentifier::ArmorIdentifier;
	void Identify(const Img& imgThre, const Img& imgGray, std::vector<ArmorPlate>& result);
};


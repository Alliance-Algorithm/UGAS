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

#include <vector>

#include "Core/Identifier/ArmorIdentifierInterface.h"
#include "Core/Identifier/NumberIdentifierInterface.h"
#include "Core/Identifier/Armor/FastStruct/FastStruct.h"

class ArmorIdentifier_V2 : public ArmorIdentifierInterface {
private:
	NumberIdentifierInterface& _numberIdentifier;

	std::queue<cv::Point> _dfsQueue;
	FastDisjointSet<cv::Point> _disjointSet;
	FastArray2d<FastDisjointSet<cv::Point>::Node*> _floodMap;
	std::vector<LightBar> _lightBarList;

	std::vector<std::vector<cv::Point>> _floodFindAreas(const cv::Mat& grayImg, int area_val, int flood_val, bool border_engulfment = false);
	void _floodPixel(const cv::Mat& grayImg, int flood_val, FastDisjointSet<cv::Point>::Node* source, int fy, int fx);
	bool _solveToLightbar(const std::vector<cv::Point>& area);
	std::vector<ArmorPlate> _matchArmorPlates(const cv::Mat& imgGray);
public:
	ArmorIdentifier_V2(NumberIdentifierInterface& numberIdentifier) : _numberIdentifier(numberIdentifier) { }

	std::vector<ArmorPlate> Identify(const cv::Mat& imgThre, const cv::Mat& imgGray);
};

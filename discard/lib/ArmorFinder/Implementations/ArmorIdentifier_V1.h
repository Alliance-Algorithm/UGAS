#pragma once
/*
Creation Date: 2022/10/13
Latest Update: 2022/10/13
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 提供一个暂时充数的实现，就叫 ArmorIdentifier Ver 1.0
*/
#include "../ArmorIdentifier.h"

class ArmorIdentifier_V1 : public ArmorIdentifier {
private:
	std::vector<LightBar> _lightBars;

	void FindLightBars(const cv::Mat& imgThre);
	std::vector<ArmorPlate> FindArmorPlates(const cv::Mat& imgGray);
public:
	ArmorIdentifier_V1(NumberIdentifier& numberIdentifier) :
		ArmorIdentifier(numberIdentifier) {}

	std::vector<ArmorPlate> Identify(const cv::Mat& imgThre, const cv::Mat& imgGray);
};

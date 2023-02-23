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

	void FindLightBars(const Img& imgThre);
	void FindArmorPlates(const Img& imgGray, std::vector<ArmorPlate>& result);
public:
	ArmorIdentifier_V1(NumberIdentifier& numberIdentifier) :
		ArmorIdentifier(numberIdentifier) {}

	void Identify(const Img& imgThre, const Img& imgGray, std::vector<ArmorPlate>& result);
};

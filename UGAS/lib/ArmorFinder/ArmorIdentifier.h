#pragma once
/*
Creation Date: 2022/10/13
Latest Update: 2022/10/13
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 提供装甲识别类的接口
*/
#include <GimbalSerial/GimbalSerialHandle.h>
#include <Common/DebugTools/DebugHeader.h>
#include "NumberIdentifier/NumberIdentifier.h"

class ArmorIdentifier {
protected:
	NumberIdentifier&		_numberIdentifier;
public:
	ArmorIdentifier(NumberIdentifier& numberIdentifier) :
		_numberIdentifier(numberIdentifier)
		{ numberIdentifier.init(static_cast<void*>(&numberIdPara)); }

	virtual void Identify(const Img& imgThre, const Img& imgGray, std::vector<ArmorPlate>& result) = 0;
};

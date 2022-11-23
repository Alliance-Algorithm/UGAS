#pragma once

#include "../NumberIdentifier.h"
#include "Common/DebugTools/DebugHeader.h"
class NumberIdentifier_V1 : public NumberIdentifier {
	void init(void*);
	short Identify(const Img& imgGray, const ArmorPlate& region);
};


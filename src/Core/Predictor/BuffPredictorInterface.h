#pragma once
/*
Creation Date: 2023/5/22
Latest Update: 2023/5/22
Developer(s): 21-WZY 21-WTR
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- interface for buff predictor
*/

#include <vector>

#include "Core/Identifier/Buff/BuffStruct.h"


class BuffPredictorInterface final {
public:
	virtual ~BuffPredictorInterface() = default;

	virtual void Predict(double deltaTimeSec) = 0;
};

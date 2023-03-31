#pragma once
/*
Creation Date: 2022/10/19
Latest Update: 2022/10/19
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 从多个目标中选择一个以跟踪
  在丢失目标后有预测跟踪迟滞
  返回值表示是否正在跟踪目标
*/
#include "Common/DebugTools/DebugHeader.h"

class TrackingStrategy {
public:
	virtual ~TrackingStrategy() = default;

	virtual int GetTargetID() = 0;
};

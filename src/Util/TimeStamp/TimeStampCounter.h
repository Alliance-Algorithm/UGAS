#pragma once
/*
Creation Date: 2022/10/12
Latest Update: 2022/10/12
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 提供获取系统时间戳的方法
*/

typedef unsigned long long TimeStamp;

class TimeStampCounter {
public:
	static TimeStamp GetTimeStamp();
};

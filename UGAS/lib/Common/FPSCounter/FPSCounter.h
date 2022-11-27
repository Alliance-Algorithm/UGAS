#pragma once
/*
Creation Date: 2022/10/16
Latest Update: 2022/10/16
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 提供统计帧数信息的方法
*/
#include <DebugSettings.h>
#include <Common/TimeStamp/TimeStampCounter.h>
#include <Common/UniversalStruct.h>

class FPSCounter {
private:
	CircularQueue<TimeStamp, MAX_FPS + 100> _timeStamps;
public:
	int Count();
	void PrintFPS(cv::Mat& img);
};

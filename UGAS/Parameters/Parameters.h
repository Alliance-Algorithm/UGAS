#pragma once
/*
Creation Date: 2022/10/11
Latest Update: 2022/10/13
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 声明并初始化所有参数
- 根据 "DebugSettings.h" 创建 Debug 拖动条
- 提供调试性函数
*/
#include <opencv.hpp>
#include "DebugSettings.h"
#include "../lib/Common/UniversalStruct.h"
#define TRACKBAR_NAME "Default TrackBars"

#if DEBUG_PARA == 1
// Debug Functions Announcement
void AddTrackbar(const cv::String& trackbarname, int* value, int count,	// for int var
	const cv::String& winname = TRACKBAR_NAME, cv::TrackbarCallback onChange = 0);
void AddTrackbar(const cv::String& trackbarname,						// for double var
	double* value, int div, int count, const cv::String& winname = TRACKBAR_NAME);
void DebugImg(const cv::String& winname, cv::InputArray mat);
#endif
// Initialize Function
void ParametersInit(const Team team);

/* Univertial Parameters */
extern Team team;
extern VIDEO_VAR_TYPE video;
extern NUM_PARA_TYPE numberIdPara;
extern int frameWidth, frameHeight;


/* Pretreat Parameters */
extern int BHmin, BHmax, BSmin, BSmax, BVmin, BVmax;
extern int RHminL, RHmaxL, RHminR, RHmaxR,
				RSmin, RSmax, RVmin, RVmax;
extern int closeCoreSize;


/* LightBar Parameters */
extern int minLightRatio, maxLightRatio;
extern int minLightAngle, maxLightAngle;


/* Armor Parameters */
extern double maxArmorLightRatio, maxdAngle,
	maxMalposition, maxLightDy, bigArmorDis;


/* Buff Parameters */


/* PNP Parameters */


/* AttitudeSolution Parameters */


/* TrackingStrategy Parameters */


/* Trajectory Parameters */



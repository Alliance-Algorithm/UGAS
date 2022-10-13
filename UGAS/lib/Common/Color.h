#pragma once
/*
Creation Date: 2022/10/13
Latest Update: 2022/10/13
Developer(s): 21-THY 21-YY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 提供一些常用的基本颜色BGR值
*/
#include <opencv.hpp>
using cv::Scalar;

#define COLOR_RED				Scalar(0	,0		,255)
#define COLOR_PINK				Scalar(196	,196	,255)
#define COLOR_DARKRED			Scalar(0	,0		,128)

#define COLOR_BLUE				Scalar(255	,0		,0)
#define COLOR_LIGHTBLUE			Scalar(255	,196	,196)
#define COLOR_DARKBLUE			Scalar(0	,0		,128)

#define COLOR_GREEN				Scalar(0	,255	,0)
#define COLOR_LIGHTGREEN		Scalar(196	,255	,196)
#define COLOR_DARKGREEN			Scalar(0	,128	,0)

#define COLOR_BLACK				Scalar(0	,0		,0)
#define COLOR_DARKGRAY			Scalar(88	,88		,88)
#define COLOR_LIGHTGRAY			Scalar(192	,192	,192)
#define COLOR_WHITE				Scalar(255	,255	,255)

#define COLOR_PURPLE			Scalar(255	,0		,255)
#define COLOR_YELLOW			Scalar(0	,255	,255)
#define COLOR_VIOLENT			Scalar(128	,0		,128)
#define COLOR_MINTGREEN			Scalar(204	,255	,0)
#define COLOR_BROWN				Scalar(0	,63		,125)
#define COLOR_MALACHITEGREEN	Scalar(128	,128	,0)
#define COLOR_EARTHYYELLOW		Scalar(0	,128	,128)
#define COLOR_ORANGE			Scalar(0	,128	,255)

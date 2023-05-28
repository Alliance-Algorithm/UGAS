#pragma once
/*
Creation Date: 2022/10/11
Latest Update: 2023/05/09
Developer(s): 21-THY 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 声明并初始化所有参数
- 根据 "DebugSettings.h" 创建 Debug 拖动条
- 提供调试性函数
- 此文件未来可能被废弃或重组
*/

#include <cstdint>

#include <vector>

#include <opencv2/opencv.hpp>

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
//void ParametersInit(const Team team);

/* Univertial Parameters */
//extern Team team;
//extern VIDEO_VAR_TYPE video;
//extern NUM_PARA_TYPE numberIdPara;
//extern int frameWidth, frameHeight;
extern cv::Point2f ROIoffset;

/* Pretreat Parameters */
extern int BHmin , BHmax , BSmin, BSmax, BVmin, BVmax;
extern int RHmaxL, RHminR, RSmin, RSmax, RVmin, RVmax;
extern int closeCoreSize;

/* LightBar Parameters */
extern int minLightRatio, maxLightRatio;
extern int minLightAngle, maxLightAngle;

/* Armor Parameters */
extern double maxArmorLightRatio, maxdAngle,
	maxMalposition, maxLightDy, bigArmorDis;

/* Buff Parameters */

/* PNP Parameters */
extern double CameraMatrixData[3][3], DistCoeffsData[1][5];
extern const cv::Mat CameraMatrix, DistCoeffs;
// TL -> BL -> BR -> TR
extern const std::vector<cv::Point3f> NormalArmor3f, LargeArmor3f;

/* AttitudeSolution Parameters */

/* TrackingStrategy Parameters */
extern int maxArmorTrackDis;
extern double keep_tracking, rotation_validity;

/* Trajectory Parameters */
extern const int iterations, Trajc_iterate;
extern const double Trajc_k, Trajc_dertaT;
extern const double angleLowest, angleHighest, angleEPS;
extern const double staticReactionTime;

namespace MathConsts {
	inline constexpr double Pi = 3.1415926535897;
	inline constexpr double G = 9.8;
};

enum class ArmorColor : uint8_t {
	Blue = 0, Red = 1
};

namespace Parameters {
	inline constexpr ArmorColor DefaultEnemyColor = ArmorColor::Blue;
	inline constexpr double DefaultBulletSpeed = 28.2;
}

/* Colors */
#define COLOR_RED				cv::Scalar(0	,0		,255)
#define COLOR_PINK				cv::Scalar(196	,196	,255)
#define COLOR_DARKRED			cv::Scalar(0	,0		,128)

#define COLOR_BLUE				cv::Scalar(255	,0		,0)
#define COLOR_LIGHTBLUE			cv::Scalar(255	,196	,196)
#define COLOR_DARKBLUE			cv::Scalar(0	,0		,128)

#define COLOR_GREEN				cv::Scalar(0	,255	,0)
#define COLOR_LIGHTGREEN		cv::Scalar(196	,255	,196)
#define COLOR_DARKGREEN			cv::Scalar(0	,128	,0)

#define COLOR_BLACK				cv::Scalar(0	,0		,0)
#define COLOR_DARKGRAY			cv::Scalar(88	,88		,88)
#define COLOR_LIGHTGRAY			cv::Scalar(192	,192	,192)
#define COLOR_WHITE				cv::Scalar(255	,255	,255)

#define COLOR_PURPLE			cv::Scalar(255	,0		,255)
#define COLOR_YELLOW			cv::Scalar(0	,255	,255)
#define COLOR_VIOLENT			cv::Scalar(128	,0		,128)
#define COLOR_MINTGREEN			cv::Scalar(204	,255	,0)
#define COLOR_BROWN				cv::Scalar(0	,63		,125)
#define COLOR_MALACHITEGREEN	cv::Scalar(128	,128	,0)
#define COLOR_EARTHYYELLOW		cv::Scalar(0	,128	,128)
#define COLOR_ORANGE			cv::Scalar(0	,128	,255)
#define COLOR_LIME				cv::Scalar(255	,255	,0)

#include "Parameters.h"
using namespace cv;

#if DEBUG_PARA == 1
// Function Implementations
void AddTrackbar(const cv::String& trackbarname, int* value, int count,
	cv::TrackbarCallback onChange, const cv::String& winname)
{
	createTrackbar(trackbarname, winname, value, count, onChange);
}

void DebugImg(const cv::String& winname, cv::InputArray mat) {
	imshow(winname, mat); waitKey(1);
}
#endif

void ParametersInit(const Team team) {
#if DEBUG_PARA == 1
	/// Special init for different team
	switch (team) {
	case Red: // ===== BULE Light =====
		AddTrackbar("BHmin", &BHmin, 180);
		AddTrackbar("BHmax", &BHmax, 180);
		AddTrackbar("BSmin", &BSmin, 255);
		AddTrackbar("BSmax", &BSmax, 255);
		AddTrackbar("BVmin", &BVmin, 255);
		AddTrackbar("BVmax", &BVmax, 255);
		break;
	case Blue: // ===== RED Light =====
		AddTrackbar("RHminL", &RHminL, 180);
		AddTrackbar("RHmaxL", &RHmaxL, 180);
		AddTrackbar("RHminR", &RHminR, 180);
		AddTrackbar("RHmaxR", &RHmaxR, 180);
		AddTrackbar("RSmin", &RSmin, 255);
		AddTrackbar("RSmax", &RSmax, 255);
		AddTrackbar("RVmin", &RVmin, 255);
		AddTrackbar("RVmax", &RVmax, 255);
		break;
	default: throw "Unkown Team Id!";
	}

	/// Universal init
	// LightBar Parameters
	AddTrackbar("closeCoreSize", &closeCoreSize, 50);
	AddTrackbar("minLightRatio", &minLightRatio, 255);
	AddTrackbar("maxLightRatio", &maxLightRatio, 255);
	AddTrackbar("minLightAngle", &minLightAngle, 180);
	AddTrackbar("maxLightAngle", &maxLightAngle, 180);
	// Armor Parameters


#endif
}

/* Univertial Parameters */
Team team = Blue;
VIDEO_VAR_TYPE video = VIDEO_VAR;
NUM_PARA_TYPE numberIdPara = NUM_PARA;
int frameWidth = 0, frameHeight = 0;


/* Pretreat Parameters */
int BHmin = 80, BHmax = 100, BSmin = 220,
		BSmax = 255, BVmin = 230, BVmax = 255;
int RHminL = 0, RHmaxL = 20, RHminR = 160, RHmaxR = 180,
		RSmin = 70, RSmax = 255, RVmin = 110, RVmax = 255;
int closeCoreSize = 17;


/* LightBar Parameters */
int minLightRatio = 3, maxLightRatio = 20;
int minLightAngle = 0, maxLightAngle = 40;


/* Armor Parameters */
double maxArmorLightRatio = 1.5, maxdAngle = 9.5, \
	maxMalposition = 0.7, maxLightDy = 0.9, bigArmorDis = 5.9;


/* Buff Parameters */


/* PNP Parameters */


/* AttitudeSolution Parameters */


/* TrackingStrategy Parameters */


/* Trajectory Parameters */



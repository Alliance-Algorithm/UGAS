#include "Parameters.h"
using namespace cv;

#if DEBUG_PARA == 1
// Function Implementations
void AddTrackbar(const cv::String& trackbarname, int* value, int count,
	const cv::String& winname, cv::TrackbarCallback onChange)
{
	createTrackbar(trackbarname, winname, value, count, onChange);
}

class DoubleTrackBarCB {
	double* _value;
	int _div, _setValue;
public:
	DoubleTrackBarCB(double* value, int div, int setValue) :
		_value(value), _div(div), _setValue(setValue) {}

	static void CallBackFunc(int pos, void* data) {
		DoubleTrackBarCB* _this = static_cast<DoubleTrackBarCB*>(data);
		*(_this->_value) = static_cast<double>(pos) / _this->_div;
	}
	friend void AddTrackbar(const cv::String& trackbarname,
		double* value, int div, int count, const cv::String& winname);
};

void AddTrackbar(const cv::String& trackbarname,
	double* value, int div, int count, const cv::String& winname)
{
	// 因为只创建一次，生命周期为整个程序运行周期，这个new没有对应的delete
	// 如需修改，需要加入后即不变物理地址的容器来管理内存，否则TrackBar拿到的指针地址不安全
	DoubleTrackBarCB* doubleCB = new DoubleTrackBarCB(value, div, *value * div);
	createTrackbar(trackbarname, winname, &doubleCB->_setValue, count * div,
		DoubleTrackBarCB::CallBackFunc, doubleCB);
}

void DebugImg(const cv::String& winname, cv::InputArray mat) {
	imshow(winname, mat); waitKey(1);
}
#endif

void ParametersInit(const Team team) {
	namedWindow(TRACKBAR_NAME, WINDOW_NORMAL);
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
	AddTrackbar("maxArmorLightRatio",	&maxArmorLightRatio,	10, 20);
	AddTrackbar("maxdAngle",			&maxdAngle,				10, 20);
	AddTrackbar("maxMalposition",		&maxMalposition,		10, 10);
	AddTrackbar("maxLightDy",			&maxLightDy,			10, 10);
	AddTrackbar("bigArmorDis",			&bigArmorDis,			10, 10);


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



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
#if DEBUG_PARA == 1
	namedWindow(TRACKBAR_NAME, WINDOW_NORMAL);
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
		AddTrackbar("RHmaxL", &RHmaxL, 180);
		AddTrackbar("RHminR", &RHminR, 180);
		AddTrackbar("RSmin", &RSmin, 255);
		AddTrackbar("RSmax", &RSmax, 255);
		AddTrackbar("RVmin", &RVmin, 255);
		AddTrackbar("RVmax", &RVmax, 255);
		break;
	default: throw_with_trace(std::runtime_error, "Unkown Team Id!");
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

	// TrackingStrategy Parameters
	AddTrackbar("maxArmorTrackDis", &maxArmorTrackDis, 100);


#endif
}

/* Univertial Parameters */
Team team = Blue;
VIDEO_VAR_TYPE video = VIDEO_VAR;
NUM_PARA_TYPE numberIdPara = NUM_PARA;
int frameWidth = 0, frameHeight = 0;

/* Pretreat Parameters */
int BHmin = 90, BHmax = 120, BSmin = 200,
		BSmax = 255, BVmin = 230, BVmax = 255;
int RHmaxL = 15, RHminR = 130, RSmin = 150,
		RSmax = 200, RVmin = 120, RVmax = 255;
int closeCoreSize = 13;

/* LightBar Parameters */
int minLightRatio = 3, maxLightRatio = 20;
int minLightAngle = 0, maxLightAngle = 40;

/* Armor Parameters */
double maxArmorLightRatio = 1.5, maxdAngle = 9.5, \
	maxMalposition = 0.7, maxLightDy = 0.9, bigArmorDis = 4.5;

/* Buff Parameters */

/* PNP Parameters */
bool isLargeArmor[10];
// 注意两个相机矩阵的重标定
double CameraMatrixData[3][3] = {	1867.490995615071, 0, 469.7628091162226, \
									0, 1873.208292955122, 464.9909258534828, \
									0, 0, 1 };
double DistCoeffsData[1][5] = { 0.02620928720926602, -0.01944757276326156, \
				0.009617926923965665, 0.0004854948878220314, -2.381184392406337 };
const Mat	CameraMatrix(3, 3, CV_64F, CameraMatrixData),
			DistCoeffs(1, 5, CV_64F, DistCoeffsData);
const int	NormalArmorWidth = 134, NormalArmorHeight = 56, \
			LargerArmorWidth = 230, LargerArmorHeight = 56;
const std::vector<Point3f>
	NormalArmor3f = {
	/*Top Left*/	 Point3f(-0.5 * NormalArmorWidth,	0.5 * NormalArmorHeight ,	0.0f),	
	/*Bottom Left*/	 Point3f(-0.5 * NormalArmorWidth,	-0.5 * NormalArmorHeight,	0.0f),	
	/*Bottom Right*/ Point3f(0.5 * NormalArmorWidth ,	-0.5 * NormalArmorHeight,	0.0f),	
	/*Top Right*/	 Point3f(0.5 * NormalArmorWidth ,	0.5 * NormalArmorHeight ,	0.0f) },
	LargeArmor3f = { Point3f(-0.5 * LargerArmorWidth,	0.5 * LargerArmorHeight ,	0.0f),
					 Point3f(-0.5 * LargerArmorWidth,	-0.5 * LargerArmorHeight,	0.0f),
					 Point3f(0.5 * LargerArmorWidth ,	-0.5 * LargerArmorHeight,	0.0f),
					 Point3f(0.5 * LargerArmorWidth ,	0.5 * LargerArmorHeight ,	0.0f) };

/* AttitudeSolution Parameters */

/* TrackingStrategy Parameters */
int maxArmorTrackDis = 10;
double keep_tracking = 0.3, rotation_validity = 0.2;

/* Trajectory Parameters */
const int iterations = 2, Trajc_iterate = 2;
const double Trajc_k = 0.00001, Trajc_dertaT = 0.0001;
const double angleLowest = -30.0, angleHighest = 45.0, angleEPS = 1e-2;
const double staticReactionTime = 0.05;

#include "Parameters.h"
using namespace cv;

/* Univertial Parameters */
//Team team = Blue;
//VIDEO_VAR_TYPE video = VIDEO_VAR;
//NUM_PARA_TYPE numberIdPara = NUM_PARA;
//int frameWidth = 0, frameHeight = 0;
cv::Point2f ROIoffset;

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
maxMalposition = 0.7, maxLightDy = 0.9, bigArmorDis = 5.0;

/* Buff Parameters */

/* PNP Parameters */

// 6mm镜头
double CameraMatrixData[3][3] = {1.722231837421459e+03, 0, 7.013056440882832e+02, \
                                    0, 1.724876404292754e+03, 5.645821718351237e+02, \
                                    0, 0, 1 };
double DistCoeffsData[1][5] = { -0.064232403853946, -0.087667493884102, 0, 0, 0.792381808294582 };

// 8mm镜头(6.2 18:35)
/*double CameraMatrixData[3][3] = {2411.67435752361, 0, 706.115296931680, \
                                    0, 2413.93326236200, 606.089691712260, \
                                    0, 0, 1 };
double DistCoeffsData[1][5] = { -0.0299345399861973, 0.127101646352983, 0, 0, 0.155503759356591 };*/

// 12mm镜头(6.2 19:35)
/*double CameraMatrixData[3][3] = {3612.09388868374, 0, 713.350996606441, \
                                    0, 3615.26346146138, 590.928598586120, \
                                    0, 0, 1 };
double DistCoeffsData[1][5] = { 0.00948033127678667, -0.900070016384203, 0, 0, 12.9708921407335 };*/

const Mat    CameraMatrix(3, 3, CV_64F, CameraMatrixData), DistCoeffs(1, 5, CV_64F, DistCoeffsData);
const int    NormalArmorWidth = 134, NormalArmorHeight = 56, LargerArmorWidth = 230, LargerArmorHeight = 56;
const std::vector<Point3f>
    NormalArmor3f = {
    /*Top Left*/     Point3f(-0.5 * NormalArmorWidth,    0.5 * NormalArmorHeight ,    0.0f),
    /*Bottom Left*/     Point3f(-0.5 * NormalArmorWidth,    -0.5 * NormalArmorHeight,    0.0f),
    /*Bottom Right*/ Point3f(0.5 * NormalArmorWidth ,    -0.5 * NormalArmorHeight,    0.0f),
    /*Top Right*/     Point3f(0.5 * NormalArmorWidth ,    0.5 * NormalArmorHeight ,    0.0f) },
    LargeArmor3f = { Point3f(-0.5 * LargerArmorWidth,    0.5 * LargerArmorHeight ,    0.0f),
                     Point3f(-0.5 * LargerArmorWidth,    -0.5 * LargerArmorHeight,    0.0f),
                     Point3f(0.5 * LargerArmorWidth ,    -0.5 * LargerArmorHeight,    0.0f),
                     Point3f(0.5 * LargerArmorWidth ,    0.5 * LargerArmorHeight ,    0.0f) };

/* AttitudeSolution Parameters */

/* TrackingStrategy Parameters */
int maxArmorTrackDis = 10;
double keep_tracking = 0.3, rotation_validity = 0.2;

/* Trajectory Parameters */
constexpr int iterations = 2, Trajc_iterate = 2;
constexpr double Trajc_k = 0.00001, Trajc_dertaT = 0.0001;
constexpr double angleLowest = -30.0, angleHighest = 45.0, angleEPS = 1e-2;
constexpr double staticReactionTime = 0.05;

#include "UniversalFunctions.h"
using namespace cv;

double P2PDis(const cv::Point2f& a, const cv::Point2f& b) {
	Point2f tmp = b - a;
	return sqrt(tmp.x * tmp.x + tmp.y * tmp.y);
}

double P2PDis(const cv::Point3f& a, const cv::Point3f& b) {
	Point3f tmp = b - a;
	return sqrt(tmp.x * tmp.x + tmp.y * tmp.y + tmp.z * tmp.z);
}

double malposition(const LightBar& LBl, const LightBar& LBr) {
	Point2f axis = (LBl.top - LBl.bottom + LBr.top - LBr.bottom) / 2;
	Point2f dis  = (LBl.top + LBl.bottom - LBr.top - LBr.bottom) / 2;
	return fabs(axis.dot(dis) / axis.cross(dis));
}

double CoordinateDistance(double a, double b) {
	return sqrt(a * a + b * b);
}

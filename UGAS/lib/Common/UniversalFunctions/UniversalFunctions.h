#pragma once
#include <opencv.hpp>
#include "../UniversalStruct.h"

double P2PDis(const cv::Point2f& a, const cv::Point2f& b);
double P2PDis(const cv::Point3f& a, const cv::Point3f& b);

double malposition(const LightBar& LBl, const LightBar& LBr);

double CoordinateDistance(double a, double b);

#pragma once
#include "../TargetSolution.h"

class TargetSolution_V1 : public TargetSolution {
protected:
	cv::Vec3f SolvePNP(const ArmorPlate& armor);
public:
	TargetSolution_V1(serial::GimbalSerial& com) :
		TargetSolution(com) {}

	void Solve(const std::vector<ArmorPlate>& armors);
};

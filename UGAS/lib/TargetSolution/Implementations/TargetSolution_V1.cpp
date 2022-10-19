#include "TargetSolution_V1.h"
using namespace cv;

cv::Vec3f TargetSolution_V1::SolvePNP(const ArmorPlate& armor) {
	Vec3f res;
	Mat rvec, tvec;
	if (isLargeArmor[armor.id]) {
		solvePnP(LargeArmor3f, armor.points, CameraMatrix, DistCoeffs,
			rvec, tvec, false, SOLVEPNP_AP3P);
	}
	else {
		solvePnP(NormalArmor3f, armor.points, CameraMatrix, DistCoeffs,
			rvec, tvec, false, SOLVEPNP_AP3P);
	}
	res = Vec3f(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
	// 还有一步从相对坐标转化为绝对坐标没写
	// 根据陀螺仪的数据补偿世界偏转

	return res;
}

void TargetSolution_V1::Solve(const std::vector<ArmorPlate>& armors) {
	// 待完成
	if (armors.size()) {
		_targets.clear();
		_targets.push_back(Target(SolvePNP(armors[0])));
	}
}

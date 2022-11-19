#include "TargetSolution_V1.h"
using namespace cv;

inline bool TargetSolution_V1::isValidId(int id) {
	return 0 < id && id < 10;
}

void TargetSolution_V1::Solve(TimeStamp ImgTime, const std::vector<ArmorPlate>& armors) {
	for (auto& armor : armors) {
		if (isValidId(armor.id)) {
			robots[armor.id].Update(ImgTime, armor);
		}
		else
		{ // 幻想了一个垃圾桶机制，以后写

		}
	}
}

#include "ArmorIdentifier_V1.h"
#include "../../Common/UniversalFunctions/UniversalFunctions.h"
using namespace std;
using namespace cv;

void ArmorIdentifier_V1::FindLightBars(const Img& img) {
	static vector< vector<Point> > contours;

	contours.clear();
	_lightBars.clear();
	findContours(img, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);

	for (const auto& contour : contours) {
		if (contour.size() > 3) {
			RotatedRect rect = minAreaRect(contour);
			float ratio = max(rect.size.width, rect.size.height) / min(rect.size.width, rect.size.height);
			if (minLightRatio < ratio && ratio < maxLightRatio) {
				float angle;
				if (rect.size.width > rect.size.height)
					angle = 90 - rect.angle;
				else angle = -rect.angle;

				if (fabs(angle) < maxLightAngle) {
					Point2f rectPoints[4];
					rect.points(rectPoints);
					if (P2PDis(rectPoints[0], rectPoints[1]) > P2PDis(rectPoints[1], rectPoints[2])) {
						_lightBars.push_back({ (rectPoints[1] + rectPoints[2]) / 2,
							(rectPoints[0] + rectPoints[3]) / 2 , angle });
					}
					else {
						_lightBars.push_back({ (rectPoints[0] + rectPoints[1]) / 2,
							(rectPoints[2] + rectPoints[3]) / 2 , angle });
					}
				}
			}
		}
	}
}

void ArmorIdentifier_V1::FindArmorPlates(const Img& img, std::vector<ArmorPlate>& result) {
	result.clear();
	sort(_lightBars.begin(), _lightBars.end(),
		[&](LightBar& a, LightBar& b) {
			return a.top.x < b.top.x;
		}
	);
	int n = _lightBars.size();
	for (int i = 0; i < n; ++i) {
		float Isize = P2PDis(_lightBars[i].top, _lightBars[i].bottom);
		Point2f Icenter = (_lightBars[i].top + _lightBars[i].bottom) / 2;
		for (int j = i + 1; j < n; ++j) { // 一些筛选条件
			float Jsize = P2PDis(_lightBars[j].top, _lightBars[j].bottom);
			if (max(Isize, Jsize) / min(Isize, Jsize) > maxArmorLightRatio)		continue;
			if (fabs(_lightBars[i].angle - _lightBars[j].angle) > maxdAngle)	continue;
			if (malposition(_lightBars[i], _lightBars[j]) > maxMalposition)		continue;
			Point2f Jcenter = (_lightBars[j].top + _lightBars[j].bottom) / 2;
			if (fabs(Icenter.y - Jcenter.y) * 2 / (Isize + Jsize) > maxLightDy)	continue;
			if (P2PDis(Icenter, Jcenter) * 2 / (Isize + Jsize) > bigArmorDis)	continue;

			// 数字识别部分（暂时放这，可能会挪到运动模型那去）
			ArmorPlate armor(_lightBars[i], _lightBars[j]);
			armor.id = _numberIdentifier.Identify(img, armor);
			result.push_back(armor);
		}
	}
}

void ArmorIdentifier_V1::Identify(const Img& img, std::vector<ArmorPlate>& result) {
	FindLightBars(img);
	FindArmorPlates(img, result);
}

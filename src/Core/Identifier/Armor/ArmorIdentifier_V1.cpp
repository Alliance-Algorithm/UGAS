#include "ArmorIdentifier_V1.h"

#include "Util/Debug/DebugCanvas.h"
#include "Util/Parameter/Parameters.h"
#include "Util/Util.h"

using namespace std;
using namespace cv;


void ArmorIdentifier_V1::FindLightBars(const Mat& imgThre) {
	static vector<vector<Point>> contours;

	contours.clear();
	_lightBars.clear();
	findContours(imgThre, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);

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
						_lightBars.push_back({ (rectPoints[1] + rectPoints[2]) / 2 + ROIoffset,
							(rectPoints[0] + rectPoints[3]) / 2 + ROIoffset, angle });
					} // 在创建灯条对象时注意补偿ROI的偏移
					else {
						_lightBars.push_back({ (rectPoints[0] + rectPoints[1]) / 2 + ROIoffset,
							(rectPoints[2] + rectPoints[3]) / 2 + ROIoffset, angle });
					}
				}
			}
		}
	}
}

std::vector<ArmorPlate> ArmorIdentifier_V1::FindArmorPlates(const Mat& imgGray) {
	std::vector<ArmorPlate> result;
	sort(_lightBars.begin(), _lightBars.end(),
		[&](LightBar& a, LightBar& b) {
			return a.top.x < b.top.x;
		}
	);
	auto n = _lightBars.size();
	for (int i = 0; i < n; ++i) {
		auto Isize = P2PDis(_lightBars[i].top, _lightBars[i].bottom);
		Point2f Icenter = (_lightBars[i].top + _lightBars[i].bottom) / 2;
		for (int j = i + 1; j < n; ++j) { // 一些筛选条件
			auto Jsize = P2PDis(_lightBars[j].top, _lightBars[j].bottom);
			if (max(Isize, Jsize) / min(Isize, Jsize) > maxArmorLightRatio)		continue;
			if (fabs(_lightBars[i].angle - _lightBars[j].angle) > maxdAngle)	continue;
			if (malposition(_lightBars[i], _lightBars[j]) > maxMalposition)		continue;
			Point2f Jcenter = (_lightBars[j].top + _lightBars[j].bottom) / 2;
			if (fabs(Icenter.y - Jcenter.y) * 2 / (Isize + Jsize) > maxLightDy)	continue;
			if (P2PDis(Icenter, Jcenter) * 2 / (Isize + Jsize) > bigArmorDis)	continue;

			// 数字识别
			ArmorPlate armor(_lightBars[i], _lightBars[j]);
			armor.id = _numberIdentifier.Identify(imgGray, armor);
			result.push_back(armor);
		}
	}
	return result;
}

std::vector<ArmorPlate> ArmorIdentifier_V1::Identify(const cv::Mat& imgThre, const cv::Mat& imgGray) {
	FindLightBars(imgThre);
	auto result = FindArmorPlates(imgGray);
	if constexpr (debugCanvas.lightbar) {
		for (const auto& lightBar : _lightBars) {
			line(debugCanvas.lightbar.GetMat(), lightBar.top, lightBar.bottom, COLOR_BLUE, 5);
			circle(debugCanvas.lightbar.GetMat(), lightBar.top, 2, COLOR_ORANGE, 2);
			circle(debugCanvas.lightbar.GetMat(), lightBar.bottom, 2, COLOR_PINK, 2);
			auto angle = to_string(lightBar.angle); angle.resize(4);
			//TextFormat(angle).SetFontScale(0.5)
			//	.Draw(img, lightBar._bottom, COLOR_YELLOW, Direction::BOTTOM_RIGHT);
			putText(debugCanvas.lightbar.GetMat(), angle, lightBar.bottom, 0, 0.5, COLOR_YELLOW);
		}
	}
	if constexpr (debugCanvas.armor) {
		for (const auto& armorPlate : result) {
			const vector<Point2f>& points = armorPlate.points;
			line(debugCanvas.armor.GetMat(), points[0], points[1], COLOR_WHITE);
			line(debugCanvas.armor.GetMat(), points[1], points[2], COLOR_LIGHTGRAY);
			line(debugCanvas.armor.GetMat(), points[2], points[3], COLOR_DARKGRAY);
			line(debugCanvas.armor.GetMat(), points[3], points[0], COLOR_RED);
			circle(debugCanvas.armor.GetMat(), armorPlate.center(), 3, COLOR_GREEN, 2);
		}
	}

	return result;
}

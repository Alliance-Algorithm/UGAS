#include "ArmorIdentifier_V1.h"
#include "Common/UniversalFunctions/UniversalFunctions.h"
#include "Common/Color.h"
using namespace std;
using namespace cv;

void ArmorIdentifier_V1::FindLightBars(const Img& imgThre) {
	static vector< vector<Point> > contours;

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

void ArmorIdentifier_V1::FindArmorPlates(const Img& imgGray, std::vector<ArmorPlate>& result) {
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
			armor.id = _numberIdentifier.Identify(imgGray, armor);
			result.push_back(armor);
		}
	}
}

void ArmorIdentifier_V1::Identify(const Img& imgThre, const Img& imgGray, std::vector<ArmorPlate>& result) {
	FindLightBars(imgThre);
	FindArmorPlates(imgGray, result);
#if DEBUG_ARMOR == 1
	for (const auto& lightBar : _lightBars) {
		line(debugImg, lightBar.top, lightBar.bottom, COLOR_BLUE, 5);
		circle(debugImg, lightBar.top, 2, COLOR_ORANGE, 2);
		circle(debugImg, lightBar.bottom, 2, COLOR_PINK, 2);
		auto angle = to_string(lightBar.angle); angle.resize(4);
		//TextFormat(angle).SetFontScale(0.5)
		//	.Draw(img, lightBar._bottom, COLOR_YELLOW, Direction::BOTTOM_RIGHT);
		putText(debugImg, angle, lightBar.bottom, 0, 0.5, COLOR_YELLOW);
	}
	for (const auto& armorPlate : result) {
		const vector<Point2f>& points = armorPlate.points;
		line(debugImg, points[0], points[1], COLOR_WHITE);
		line(debugImg, points[1], points[2], COLOR_LIGHTGRAY);
		line(debugImg, points[2], points[3], COLOR_DARKGRAY);
		line(debugImg, points[3], points[0], COLOR_RED);
		circle(debugImg, armorPlate.center(), 3, COLOR_GREEN, 2);
	}
#endif
}

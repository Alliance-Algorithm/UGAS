#pragma once

#include <optional>

#include "Core/Identifier/StandaloneArmorIdentifierInterface.h"
#include "Core/Identifier/Color/ColorIdentifier_V1.h"
#include "Core/Identifier/Armor/FastStruct/FastStruct.h"
#include "Core/Identifier/Number/NullNumberIdentifier.h"
#include "Util/Parameter/Parameters.h"
#include "Util/Debug/DebugCanvas.h"
#include "Util/UtilFunctions.h"

template <typename NumberIdentifierType>
class ArmorIdentifier_V3 {
public:
	template <typename... Types>
	ArmorIdentifier_V3(Types&&... args) :
		_blueIdentifier(228.0f), _redIdentifier(11.0f), _numberIdentifier(std::forward<Types>(args)...) { }

	ArmorIdentifier_V3(const ArmorIdentifier_V3&) = delete;
	ArmorIdentifier_V3(ArmorIdentifier_V3&&) = delete;

	std::vector<ArmorPlate> Identify(const cv::Mat& img, ArmorColor targetColor) {
		cv::Mat imgThre, imgGray;
		cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);
		cv::threshold(imgGray, imgThre, 150, 255, cv::THRESH_BINARY);

		if constexpr (debugCanvas.pretreat) {
			auto imgColor = _pretreat(img, targetColor);
			debugCanvas.pretreat.LoadMat(imgColor);
		}

		std::vector<std::vector<cv::Point>> contours;
		std::vector<LightBar> lightBars;
		std::vector<ArmorPlate> result;
		cv::findContours(imgThre, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

		int i = 0;
		for (const auto& contour : contours) {
			if (auto&& lightBarOpt = _solveToLightbar(img, contour, targetColor)) {
				lightBars.push_back(*lightBarOpt);

				std::sort(lightBars.begin(), lightBars.end(),
					[](LightBar& a, LightBar& b) {
					return a.top.x < b.top.x;
				});

				if constexpr (debugCanvas.lightbar) {
					auto& lightBar = *lightBarOpt;
					cv::drawContours(debugCanvas.lightbar.GetMat(), contours, i, COLOR_RED, 2);
					line(debugCanvas.lightbar.GetMat(), lightBar.top, lightBar.bottom, COLOR_BLUE, 5);
					circle(debugCanvas.lightbar.GetMat(), lightBar.top, 2, COLOR_ORANGE, 2);
					circle(debugCanvas.lightbar.GetMat(), lightBar.bottom, 2, COLOR_PINK, 2);
				}
			}
			++i;
		}

		size_t&& lightBarsSize = lightBars.size();
		for (size_t i = 0; i < lightBarsSize; ++i) {
			float Isize = P2PDis(lightBars[i].top, lightBars[i].bottom);
			cv::Point2f Icenter = (lightBars[i].top + lightBars[i].bottom) / 2;
			for (size_t j = i + 1; j < lightBarsSize; ++j) { // 一些筛选条件

				float Jsize = P2PDis(lightBars[j].top, lightBars[j].bottom);
				if (max(Isize, Jsize) / min(Isize, Jsize) > maxArmorLightRatio) continue;
				if (fabs(lightBars[i].angle - lightBars[j].angle) > maxdAngle) continue;
				if (malposition(lightBars[i], lightBars[j]) > maxMalposition) continue;
				cv::Point2f Jcenter = (lightBars[j].top + lightBars[j].bottom) / 2;
				if (fabs(Icenter.y - Jcenter.y) * 2 / (Isize + Jsize) > maxLightDy)	continue;
				if (P2PDis(Icenter, Jcenter) * 2 / (Isize + Jsize) > bigArmorDis) continue;

				ArmorPlate armor(lightBars[i], lightBars[j]);
				if (_numberIdentifier.Identify(imgGray, armor))
					result.push_back(armor);
			}
		}

		if constexpr (debugCanvas.armor) {
			for (const auto& armorPlate : result) {
				const std::vector<cv::Point2f>& points = armorPlate.points;
				line(debugCanvas.armor.GetMat(), points[0], points[1], COLOR_WHITE);
				line(debugCanvas.armor.GetMat(), points[1], points[2], COLOR_LIGHTGRAY);
				line(debugCanvas.armor.GetMat(), points[2], points[3], COLOR_DARKGRAY);
				line(debugCanvas.armor.GetMat(), points[3], points[0], COLOR_RED);
				circle(debugCanvas.armor.GetMat(), armorPlate.center(), 3, COLOR_GREEN, 2);
			}
		}

		return result;
	}

private:
	ColorIdentifier_V1 _blueIdentifier, _redIdentifier;
	NumberIdentifierType _numberIdentifier;

	cv::Mat _pretreat(const cv::Mat& src, ArmorColor targetColor) const {
		cv::Mat dst;
		dst.create(src.size(), CV_8UC1);

		cv::Size sz = src.size();
		const uchar* srcBuf = src.data;
		uchar* dstBuf = dst.data;
		size_t srcstep = src.step, dststep = dst.step;

		if (src.isContinuous() && dst.isContinuous()) {
			sz.width *= sz.height;
			sz.height = 1;
		}

		auto& colorIdentifier = targetColor == ArmorColor::Blue ? _blueIdentifier : _redIdentifier;
		for (; sz.height--; srcBuf += srcstep, dstBuf += dststep) {
			auto src = srcBuf;
			auto dst = dstBuf;
			for (int i = 0; i < sz.width; i += 1, src += 3) {
				dst[i] = static_cast<uchar>(colorIdentifier.Identify(src));
			}
		}

		return dst;
	}


	std::optional<LightBar> _solveToLightbar(const cv::Mat& img, const std::vector<cv::Point>& contour, ArmorColor targetColor) {
		auto&& contourSize = contour.size();
		if (contourSize >= 5) {

			static float scoreMap[256];
			float confidence = 0.0f;
			scoreMap[static_cast<size_t>(ColorConfidence::NotCredible)] = 0.0f;
			scoreMap[static_cast<size_t>(ColorConfidence::CredibleOneChannelOverexposure)] = 1.0f / contourSize;
			scoreMap[static_cast<size_t>(ColorConfidence::CredibleTwoChannelOverexposure)] = 0.5f / contourSize;
			scoreMap[static_cast<size_t>(ColorConfidence::CredibleThreeChannelOverexposure)] = 0.2f / contourSize;

			auto& colorIdentifier = targetColor == ArmorColor::Blue ? _blueIdentifier : _redIdentifier;
			for (const auto& point : contour) {
				auto c = reinterpret_cast<const uchar*>(&img.at<cv::Vec3b>(point));
				confidence += scoreMap[static_cast<size_t>(colorIdentifier.Identify(c))];
			}

			if (confidence > 0.5f) {
				constexpr int angleRange = 30;
				auto box = cv::minAreaRect(contour);

				if constexpr (debugCanvas.lightbar) {
					auto&& angle = std::to_string(static_cast<int>(confidence * 100 + 0.5));
					putText(debugCanvas.lightbar.GetMat(), angle, cv::Point2f(box.center.x + 10, box.center.y), 0, 0.5, COLOR_YELLOW);
				}

				cv::Point2f contour[4];
				box.points(contour);
				auto diff = box.size.width - box.size.height;
				if (diff > 0) {     //旋转前，矩形横放
					float angle = fmod(box.angle + 360, 360);
					if (90 - angleRange < angle && angle < 90 + angleRange) {
						return LightBar((contour[0] + contour[1]) / 2 + ROIoffset, (contour[2] + contour[3]) / 2 + ROIoffset, 0);
					}
					else if (270 - angleRange < angle && angle < 270 + angleRange) {
						return LightBar((contour[2] + contour[3]) / 2 + ROIoffset, (contour[0] + contour[1]) / 2 + ROIoffset, 0);
					}
				}
				else if (diff < 0) {  //旋转前，矩形横放
					float angle = fmod(box.angle + 360 + 90, 360);
					if (90 - angleRange < angle && angle < 90 + angleRange) {
						return LightBar((contour[1] + contour[2]) / 2 + ROIoffset, (contour[0] + contour[3]) / 2 + ROIoffset, 0);
					}
					else if (270 - angleRange < angle && angle < 270 + angleRange) {
						return LightBar((contour[0] + contour[3]) / 2 + ROIoffset, (contour[1] + contour[2]) / 2 + ROIoffset, 0);
					}
				}
			}
		}
		return std::nullopt;
	}
};
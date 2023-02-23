#pragma once
/*
Creation Date: 2023/1/4
Latest Update: 2023/1/4
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 按钮类，可添加进MatForm
*/
#include <opencv2/opencv.hpp>
#include "RectangleControl.hpp"


class ButtonControl : public RectangleControl {
private:
public:
	std::string Text;
	int FontFace = cv::FONT_HERSHEY_PLAIN;
	double FontScale = 1;

	using RectangleControl::RectangleControl;

	void Draw() {
		RectangleControl::Draw();
		int baseLine;
		cv::Size textSize = cv::getTextSize(Text, FontFace, FontScale, Thickness, &baseLine);
		cv::Point origin = BoundRect.tl();
		origin.x += BoundRect.width / 2 - textSize.width / 2;
		origin.y += BoundRect.height / 2 + textSize.height / 2;
		cv::putText(*_parent, Text, origin, FontFace, FontScale, ForeColor);
	};
};
#pragma once
/*
Creation Date: 2023/1/4
Latest Update: 2023/1/4
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 一个带有可点击控件(Control)的Mat类
*/
#include <opencv2/opencv.hpp>
#include "MatControl.hpp"

class MatControl;

class MatForm : public cv::Mat {
private:
	cv::String _title;
	std::vector<MatControl*> _controls;

	inline void Draw() {
		for (auto control : _controls) {
			control->Draw();
		}
	}

public:
	MatForm(const char* title) {
		_title = title;
	}

	const std::vector<MatControl*>& Controls = _controls;

	inline void LoadMat(const Mat& img) {
		static_cast<cv::Mat&>(*this) = img.clone();
	}

	inline void AddControl(MatControl* control) {
		control->SetParent(this);
		_controls.push_back(control);
	}

	inline void Show() {
		Draw();
		cv::imshow(_title, *this);
		cv::setMouseCallback(_title, OnMouse, (void*)this);
		cv::waitKey(1);
	}

	static void OnMouse(int event, int x, int y, int flags, void* param) {
		//std::cout << x << ' ' << y << std::endl;
		if (event == cv::EVENT_LBUTTONDOWN || event == cv::EVENT_LBUTTONUP || event == cv::EVENT_LBUTTONDBLCLK) {
			MatForm* _this = static_cast<MatForm*>(param);
			for (auto i = _this->_controls.begin(); i != _this->_controls.end(); ++i) {
				MatControl* control = *i;
				//std::cout << x << ' ' << y << std::endl;
				//std::cout << control->BoundRect.x << ' ' << control->BoundRect.y << ' ' << control->BoundRect.width << ' ' << control->BoundRect.height << std::endl;
				if (control->BoundRect.x <= x && x <= control->BoundRect.x + control->BoundRect.width &&
					control->BoundRect.y <= y && y <= control->BoundRect.y + control->BoundRect.height) {
					if (event == cv::EVENT_LBUTTONDOWN || event == cv::EVENT_LBUTTONDBLCLK) {
						if (control->OnMouseDown)
							control->OnMouseDown();
					}
					else if (event == cv::EVENT_LBUTTONUP) {
						if (control->OnMouseUp)
							control->OnMouseUp();
					}
				}
			}
		}
	}

};
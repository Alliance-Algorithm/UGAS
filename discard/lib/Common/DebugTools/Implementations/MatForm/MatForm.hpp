#pragma once
/*
Creation Date: 2023/1/4
Latest Update: 2023/1/4
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 一个带有可点击控件(Control)的Mat类
- 带有一个全局链表，可以通过MatForm::ShowAll显示所有实例
- 由于全局链表存在，构造和析构函数并非线程安全，只能在同一线程内构造和析构所有实例
*/
#include <opencv2/opencv.hpp>
#include "MatControl.hpp"

class MatControl;

class MatForm : public cv::Mat {
private:
	static std::list<MatForm*> _instantiatedList;

	std::string _title;
	std::list<MatForm*>::const_iterator _self;

	std::vector<MatControl*> _controls;

	inline void Draw() {
		for (const auto& control : _controls) {
			control->Draw();
		}
	}

public:
	MatForm() = delete;

	MatForm(const char* title) {
		std::cout << title << std::endl;
		_title = title;
		_instantiatedList.push_back(this);
		_self = _instantiatedList.cend();
		--_self;
	}

	~MatForm() {
		_instantiatedList.erase(_self);
	}

	const std::vector<MatControl*>& Controls = _controls;

	void LoadMat(Mat& img) {
		static_cast<cv::Mat&>(*this) = img;
	}

	void LoadMat(Mat&& img) {
		//Note: cv::Mat自带引用计数，因此Copy assignment和Move assignment在代码上无区别
		static_cast<cv::Mat&>(*this) = img;
	}

	void AddControl(MatControl* control) {
		control->SetParent(this);
		_controls.push_back(control);
	}

	void Show() {
		Draw();
		cv::imshow(_title, *this);
		cv::setMouseCallback(_title, OnMouse, (void*)this);
	}

	static void ShowAll() {
		for (const auto& it : _instantiatedList) {
			it->Show();
		}
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
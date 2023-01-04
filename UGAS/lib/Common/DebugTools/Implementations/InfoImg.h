#pragma once
/*
Creation Date: 2022/11/20
Latest Update: 2023/1/4
Developer(s): 21-THY 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 用于管理、显示调试图像
*/
#include <opencv2/opencv.hpp>
#include <DebugSettings.h>
#include <Common/Color.h>
#include "MatForm/MatForm.hpp"
#include "MatForm/RectangleControl.hpp"
#include "MatForm/ButtonControl.hpp"


class InfoImg : public MatForm {
private:
	RectangleControl _ROI;
	ButtonControl _btnForward10Frame, _btnForwardFrame, _btnPause, _btnNextFrame, _btnNext10Frame;

	int _lastFrameRows = 0, _lastFrameCols = 0;

public:
	struct FrameHandler {
		bool Paused;
		int FrameAdjust;

		FrameHandler() {
			Paused = false;
			FrameAdjust = 0;
		}
	} DebugFrameHandler;

	InfoImg() : MatForm("InfoImg") {
		_btnForward10Frame.Text = "<<";
		_btnForward10Frame.OnMouseDown = [this] {
			DebugFrameHandler.FrameAdjust -= 10;
		};
		AddControl(&_btnForward10Frame);

		_btnForwardFrame.Text = "<";
		_btnForwardFrame.OnMouseDown = [this] {
			DebugFrameHandler.FrameAdjust--;
		};
		AddControl(&_btnForwardFrame);

		_btnPause.Text = "Pause";
		_btnPause.OnMouseDown = [this] {
			if (DebugFrameHandler.Paused) {
				DebugFrameHandler.Paused = false;
				_btnPause.Text = "Pause";
			}
			else {
				DebugFrameHandler.Paused = true;
				_btnPause.Text = "Play";
			}
		};
		AddControl(&_btnPause);

		_btnNextFrame.Text = ">";
		_btnNextFrame.OnMouseDown = [this] {
			DebugFrameHandler.FrameAdjust++;
		};
		AddControl(&_btnNextFrame);

		_btnNext10Frame.Text = ">>";
		_btnNext10Frame.OnMouseDown = [this] {
			DebugFrameHandler.FrameAdjust += 10;
		};
		AddControl(&_btnNext10Frame);

		_ROI.ForeColor = COLOR_YELLOW;
		AddControl(&_ROI);
	}

	void Load(const Mat& img, const cv::Rect& roiRect) {
		MatForm::LoadMat(img);
		if (_lastFrameRows != rows || _lastFrameRows != cols) {
			//更新按钮位置
			_btnForward10Frame.BoundRect = cv::Rect(cols - 300, rows - 60, 60, 60);
			_btnForwardFrame.BoundRect = cv::Rect(cols - 240, rows - 60, 60, 60);
			_btnPause.BoundRect = cv::Rect(cols - 180, rows - 60, 60, 60);
			_btnNextFrame.BoundRect = cv::Rect(cols - 120, rows - 60, 60, 60);
			_btnNext10Frame.BoundRect = cv::Rect(cols - 60, rows - 60, 60, 60);
		}
		_ROI.BoundRect = roiRect;
	}
};

extern InfoImg debugImg;
//#if DEBUG_IMG == 1
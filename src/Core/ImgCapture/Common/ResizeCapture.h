#pragma once
/*
Creation Date: 2023/03/21
Latest Update: 2023/03/21
Developer(s): 22-QZH
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 提供对任意ImgCapture类的封装，在原Capture类的基础上增加更改大小功能
- 使用方法(以CVVideoCapture类为例):
- ResizeCapture<CVVideoCapture> capture(0.5, "Blue_4.mp4");                //把原图像等比缩放到原来的0.5倍
- ResizeCapture<CVVideoCapture> capture(cv::Size(640, 480), "Blue_4.mp4"); //把原图像缩放到(640, 480)大小
*/

#include <opencv2/opencv.hpp>

template<typename CaptureType>
class ResizeCapture : public CaptureType {
public:
	double _ratio;
	cv::Size _targetSize;

	ResizeCapture() = delete;

	template<typename... Types>
	ResizeCapture(double ratio, Types&&... args) : CaptureType(std::forward<Types>(args)...) {
		_ratio = ratio;
		_targetSize = cv::Size(0, 0);
	}

	template<typename... Types>
	ResizeCapture(cv::Size targetSize, Types&&... args) : CaptureType(std::forward<Types>(args)...) {
		_ratio = 0.0;
		_targetSize = targetSize;
	}

	ResizeCapture(const ResizeCapture&) = delete;
	ResizeCapture(ResizeCapture&&) = delete;


	std::tuple<cv::Mat, TimeStamp> Read() override {
		auto tuple = CaptureType::Read();
		auto& [mat, timestamp] = tuple;
		cv::resize(mat, mat, _targetSize, _ratio, _ratio);
		return tuple;
	}
};

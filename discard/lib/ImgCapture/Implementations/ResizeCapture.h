#pragma once
#include "CVVideoCapture.h"

template<typename CaptureType>
class ResizeCapture : public CaptureType {
public:
	double _ratio;
	cv::Size _targetSize;

	ResizeCapture() = delete;

	template<typename... Types>
	explicit ResizeCapture(double ratio, Types... args) : CaptureType(args...) {
		_ratio = ratio;
		_targetSize = cv::Size(0, 0);
	}

	template<typename... Types>
	explicit ResizeCapture(cv::Size targetSize, Types... args) : CaptureType(args...) {
		_ratio = 0.0;
		_targetSize = targetSize;
	}


	std::tuple<cv::Mat, TimeStamp> Read() {
		auto tuple = CaptureType::Read();
		auto& [mat, timestamp] = tuple;
		cv::resize(mat, mat, _targetSize, _ratio, _ratio);
		return tuple;
	}
};


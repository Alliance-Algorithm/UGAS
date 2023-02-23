#include "CVVideoFrameCapture.h"

void CVVideoFrameCapture::init(void* fileName) {
	open(*(const char**)fileName);
	if (!isOpened()) {
		open(std::string("resources/") + *(const char**)fileName);
		if (!isOpened())
			throw_with_trace(std::runtime_error, "Fail to open.");
	}
}

void CVVideoFrameCapture::read(Img& img) {
	if (img.empty() || cv::waitKey(1000. / 30.) == ' ') {
		VideoCapture::read(_lastFrame);
		if (_lastFrame.empty())
			throw_with_trace(std::runtime_error, "Read empty img!");
	}
	img = _lastFrame.clone();
	img.timeStamp = TimeStampCounter::GetTimeStamp();
}

#include "CVVideoCapture.h"
#include <Common/TimeStamp/TimeStampCounter.h>

void CVVideoCapture::init(void* fileName) {
	VideoCapture::open(*(const char**)fileName);
	if (!isOpened()) {
		VideoCapture::open(std::string("resources/") + *(const char**)fileName);
		if (!isOpened())
			throw_with_trace(std::runtime_error, "Fail to open.");
	}
	_frameCount = VideoCapture::get(cv::CAP_PROP_FRAME_COUNT);
	if (_frameCount == 0)
		throw_with_trace(std::runtime_error, "Read empty video!")
}

void CVVideoCapture::read(Img& img) {
	VideoCapture::read(img);
	if (img.empty()) {
		VideoCapture::set(cv::CAP_PROP_POS_FRAMES, 0);
		VideoCapture::read(img);
		if (img.empty()) {
			throw_with_trace(std::runtime_error, "Read empty frame!")
		}
	}
	img.timeStamp = TimeStampCounter::GetTimeStamp();
}

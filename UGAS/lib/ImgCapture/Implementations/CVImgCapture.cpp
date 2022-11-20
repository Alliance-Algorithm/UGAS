#include "CVImgCapture.h"
#include "Common/TimeStamp/TimeStampCounter.h"

void CVImgCapture::init(void* fileName) {
	open(*(const char**)fileName);
	if (!isOpened()) {
		open(std::string("resources/") + *(const char**)fileName);
		if (!isOpened())
			throw_with_trace(std::runtime_error, "Fail to open.");
	}
	_startTime = TimeStampCounter::GetTimeStamp();
}

void CVImgCapture::read(Img& img) {
	VideoCapture::read(img);
	// 不知道这玩意好不好用
	img.timeStamp = get(cv::CAP_PROP_POS_MSEC) + _startTime;
}

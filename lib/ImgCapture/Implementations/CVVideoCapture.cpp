#include "CVVideoCapture.h"
#include <Common/TimeStamp/TimeStampCounter.h>

void CVVideoCapture::init(void* fileName) {
	VideoCapture::open(*(const char**)fileName);
	if (!isOpened()) {
		VideoCapture::open(std::string("resources/") + *(const char**)fileName);
		if (!isOpened())
			throw_with_trace(std::runtime_error, "Fail to open.");
	}
	_frameCount = static_cast<int>(VideoCapture::get(cv::CAP_PROP_FRAME_COUNT));
	if (_frameCount == 0)
		throw_with_trace(std::runtime_error, "Read empty video!")
}

void CVVideoCapture::read(Img& img) {
	int frameAdjust = debugImg.DebugFrameHandler.FrameAdjust;
	debugImg.DebugFrameHandler.FrameAdjust = 0;
	if (debugImg.DebugFrameHandler.Paused)
		--frameAdjust;
	if (frameAdjust != 0) {
		int frameIndex = static_cast<int>(VideoCapture::get(cv::CAP_PROP_POS_FRAMES)) + frameAdjust;
		frameIndex %= _frameCount;
		VideoCapture::set(cv::CAP_PROP_POS_FRAMES, frameIndex);
	}	

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

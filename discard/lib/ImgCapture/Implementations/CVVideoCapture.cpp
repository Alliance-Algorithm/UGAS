#include "CVVideoCapture.h"
#include <Common/TimeStamp/TimeStampCounter.h>

CVVideoCapture::CVVideoCapture(const std::string& fileName) {
	VideoCapture::open(fileName);
	if (!isOpened()) {
		VideoCapture::open(std::string("resources/") + fileName);
		if (!isOpened())
			throw_with_trace(std::runtime_error, "Fail to open.");
	}
	_frameCount = static_cast<int>(VideoCapture::get(cv::CAP_PROP_FRAME_COUNT));
	if (_frameCount == 0)
		throw_with_trace(std::runtime_error, "Read empty video!")
}

std::tuple<cv::Mat, TimeStamp> CVVideoCapture::Read() {
	int frameAdjust = 0;// debugImg.DebugFrameHandler.FrameAdjust;
	//debugImg.DebugFrameHandler.FrameAdjust = 0;
	//if (debugImg.DebugFrameHandler.Paused)
	//	--frameAdjust;
	if (frameAdjust != 0) {
		int frameIndex = static_cast<int>(VideoCapture::get(cv::CAP_PROP_POS_FRAMES)) + frameAdjust;
		frameIndex %= _frameCount;
		VideoCapture::set(cv::CAP_PROP_POS_FRAMES, frameIndex);
	}	

	std::tuple<cv::Mat, TimeStamp> result;
	auto& [img, timestamp] = result;
	VideoCapture::read(img);
	if (img.empty()) {
		VideoCapture::set(cv::CAP_PROP_POS_FRAMES, 0);
		VideoCapture::read(img);
		if (img.empty()) {
			throw_with_trace(std::runtime_error, "Read empty frame!")
		}
	}
	timestamp = TimeStampCounter::GetTimeStamp();
	return result;
}

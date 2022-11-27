#include "CVCameraCapture.h"
#include <Common/TimeStamp/TimeStampCounter.h>

void CVCameraCapture::init(void* camIndex) {
	open(*(int*)camIndex);
	if (!isOpened())
		throw_with_trace(std::runtime_error, "Fail to open.");
}

void CVCameraCapture::read(Img& img) {
	VideoCapture::read(img);
	if (img.empty())
		throw_with_trace(std::runtime_error, "Read empty img!");
	img.timeStamp = TimeStampCounter::GetTimeStamp();
}

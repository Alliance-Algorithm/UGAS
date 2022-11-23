#include "CVVideoCapture.h"
#include "Common/TimeStamp/TimeStampCounter.h"

void CVVideoCapture::init(void* fileName) {
	open(*(const char**)fileName);
	if (!isOpened()) {
		open(std::string("resources/") + *(const char**)fileName);
		if (!isOpened())
			throw_with_trace(std::runtime_error, "Fail to open.");
	}
}

void CVVideoCapture::read(Img& img) {
	VideoCapture::read(img);
	if (img.empty())
		throw_with_trace(std::runtime_error, "Read empty img!");
	img.timeStamp = TimeStampCounter::GetTimeStamp();
}

#include "CVVideoCapture.h"
#include "Common/TimeStamp/TimeStampCounter.h"

void CVVideoCapture::init(void* camIndex) {
	open(*(int*)camIndex);
	if (!isOpened())
		throw_with_trace(std::runtime_error, "Fail to open.");
}

void CVVideoCapture::read(Img& img) {
	VideoCapture::read(img);
	img.timeStamp = TimeStampCounter::GetTimeStamp();
}

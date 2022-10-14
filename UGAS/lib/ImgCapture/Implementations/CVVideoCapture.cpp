#include "CVVideoCapture.h"

void CVVideoCapture::init(void* camIndex) {
	open(*(int*)camIndex);
	if (!isOpened())
		throw "<CVImgCapture::init> Fail to open.";
}

void CVVideoCapture::read(Img& img) {
	VideoCapture::read(img);
	// 不知道这玩意好不好用
	img.timeStamp = get(cv::CAP_PROP_POS_MSEC);
}

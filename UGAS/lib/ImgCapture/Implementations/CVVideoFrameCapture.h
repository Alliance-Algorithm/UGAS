#pragma once
/*
Creation Date: 2022/11/23
Latest Update: 2022/11/23
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 包装OpenCV自带VideoCapture，使其符合ImgCapture的接口
- 仅包含视频读入，按空格跳转下一帧
*/
#include "../ImgCapture.h"

class CVVideoFrameCapture : public ImgCapture, private cv::VideoCapture {
private:
	cv::Mat _lastFrame;
public:
	virtual void init(void* fileName);
	virtual void read(Img& img);
};

#pragma once
/*
Creation Date: 2022/10/12
Latest Update: 2022/10/12
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 包装OpenCV自带VideoCapture，使其符合ImgCapture的接口
- 仅包含视频读入
*/
#include "../ImgCapture.h"

class CVVideoCapture : public ImgCapture, private cv::VideoCapture {
private:
	int _frameCount;

public:
	virtual void init(void* fileName);
	virtual void read(Img& img);
};

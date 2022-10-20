#pragma once
/*
Creation Date: 2022/10/12
Latest Update: 2022/10/12
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 定义获取图像类的通用接口
Class public functions:
- init(void*)
	初始化图像输入
- read(Img&)
	读取一帧图像，通过修改引用的方式返回结果
*/
#include <opencv2/opencv.hpp>
#include "../../lib/Common/UniversalStruct.h"
#include "../Common/DebugTools/DebugHeader.h"

class ImgCapture {
public:
	virtual void init(void*) = 0;
	virtual void read(Img&) = 0;
};

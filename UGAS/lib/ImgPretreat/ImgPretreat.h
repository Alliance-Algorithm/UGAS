#pragma once
/*
Creation Date: 2022/10/11
Latest Update: 2022/10/11
Developer(s): 21 THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 提供图片预处理类的接口
*/
#include "../GimbalSerial/GimbalSerial.h"

class ImgPretreat {
protected:
	serial::GimbalSerial& _com;
public:
	ImgPretreat(serial::GimbalSerial& com) :_com(com) {}

	virtual void GetPretreated(Img& img) = 0;
};

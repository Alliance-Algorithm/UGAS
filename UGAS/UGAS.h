#pragma once
/*
Creation Date: 2022/10/11
Latest Update: 2022/10/11
Developer(s): 21 THY
(C)Copyright: NJUST.Alliance - All rights reserved
Class public functions:
- initial:
	initialize for UGAS
- always:
	work progress in forever loop
*/
#include <opencv.hpp>

// main progress for UGAS
class UGAS {
private:

public:
	UGAS() {}
	virtual ~UGAS() {}

	void initial();
	void always();
};

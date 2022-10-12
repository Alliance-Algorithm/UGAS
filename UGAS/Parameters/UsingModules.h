#pragma once
/*
Creation Date: 2022/10/11
Latest Update: 2022/10/11
Developer(s): 21 THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 引用包含所有使用的模块的实现
- 定义使用类型的宏
*/
#include "../lib/GimbalSerial/Implementations/Windows/WindowsGimbalSerial.h"
#include "../lib/ImgCapture/Implementations/CVVideoCapture.h"

#define GIMBAL_SERIAL	serial::WindowsGimbalSerial
#define IMG_CAPTURE		CVVideoCapture

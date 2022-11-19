#pragma once
/*
Creation Date: 2022/10/11
Latest Update: 2022/10/13
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 引用包含所有使用的模块的实现
- 定义使用类型的宏
*/
#ifdef _WIN32
#include "GimbalSerial/Implementations/Windows/WindowsGimbalSerial.h"
#define GIMBAL_SERIAL	serial::WindowsGimbalSerial
#else
#include "GimbalSerial/Implementations/Linux/LinuxGimbalSerial.h"
#define GIMBAL_SERIAL	serial::LinuxGimbalSerial
#endif
#include "ImgCapture/Implementations/CVImgCapture.h"
#include "ImgPretreat/Implementations/ImgPretreat_V1.h"
#include "ArmorFinder/Implementations/ArmorIdentifier_V1.h"
#include "ArmorFinder/NumberIdentifier/Implementations/NullNumberIdentifier.h"
#include "TargetSolution/Implementations/TargetSolution_V1.h"
#include "TrackingStrategy/Implementations/TrackingStrategy_V1.h"
#include "Trajectory/Implementations/Trajectory_FEM.h"

#define IMG_CAPTURE		CVImgCapture
#define IMG_PRETREAT	ImgPretreat_V1
#define ARMOR_IDENTIFY	ArmorIdentifier_V1
#define NUMBER_IDENTIFY	NullNumberIdentifier
#define TARGET_SOLUTION TargetSolution_V1
#define TRACK_STRATEGY  TrackingStrategy_V1
#define TRAJECTORY		Trajectory_FEM

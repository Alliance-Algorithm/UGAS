#pragma once
/*
Creation Date: 2022/10/11
Latest Update: 2022/10/13
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 包含所有 UGAS 直接需要的头文件
  包括外部库文件和自定义库文件
*/

// External Library Files
#include <opencv2/opencv.hpp>

// Custom Library Files
#include "DebugSettings.h"
#include "Parameters.h"
#include "UsingModules.h"

#include "GimbalSerial/GimbalSerial.h"
#include "ImgCapture/ImgCapture.h"
#include "ImgPretreat/ImgPretreat.h"
#include "ArmorFinder/ArmorIdentifier.h"
#include "TargetSolution/TargetSolution.h"
#include "TrackingStrategy/TrackingStrategy.h"
#include "Trajectory/Trajectory.h"

#include "Common/FPSCounter/FPSCounter.h"

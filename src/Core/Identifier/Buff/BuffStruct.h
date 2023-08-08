#pragma once
/*
Creation Date: 2023/08/04
Latest Update: 2023/08/04
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 大符数据结构
*/

#include <opencv2/opencv.hpp>

#include "Core/Transformer/Tree.h"

struct BuffPlate {
    // bl br tr tl
    std::vector<cv::Point2f> points;
};

struct BuffPlate3d {
    GimbalGyro::Position position;
    GimbalGyro::Rotation rotation;
};

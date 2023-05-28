#pragma once
/*
Creation Date: 2023/1/4
Latest Update: 2023/1/4
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 矩形类，可添加进MatForm
*/

#include <opencv2/opencv.hpp>

#include "MatControl.hpp"
#include "Util/Parameter/Parameters.h"


class RectangleControl : public MatControl {
private:

public:
    int Thickness = 1;
    cv::Scalar ForeColor = COLOR_WHITE;

    using MatControl::MatControl;

    void Draw() const {
        cv::rectangle(*_parent, BoundRect, ForeColor, Thickness);
    }
};
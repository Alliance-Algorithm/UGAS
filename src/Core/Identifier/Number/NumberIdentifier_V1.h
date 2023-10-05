#pragma once
/*
Creation Date: 2022/11/23
Latest Update: 2022/11/23
Developer(s): 22-Qzh 22-Ljc
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 数字识别V1版本
- 使用OTSU和CNN
*/

#include <opencv2/opencv.hpp>

#include "Core/Identifier/Armor/ArmorStruct.h"
#include "Core/Identifier/NumberIdentifierInterface.h"

class NumberIdentifier_V1 {
private:
    cv::dnn::Net _net;
public:
    explicit NumberIdentifier_V1(const char* model);
    NumberIdentifier_V1(const NumberIdentifier_V1&) = delete;
    NumberIdentifier_V1(NumberIdentifier_V1&&) = delete;

    bool Identify(const cv::Mat& imgGray, ArmorPlate& armor);
};

#pragma once
/*
Creation Date: 2022/11/23
Latest Update: 2022/11/23
Developer(s): 22-Qzh 22-Ljc
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 数字识别V1版本
- 使用OTSU和CNN
- 加入了模板匹配区分正负样本功能，具体情况需要再测，我写的是先用神经网络识别，后模板匹配，如果上车效果不好，需要将二者调换顺序，对于噪声较大图片，模板匹配的得分一般在0.3-0.5之间
*/

#include <opencv2/opencv.hpp>
#include <vector>

#include "Core/Identifier/Armor/ArmorStruct.h"
#include "Core/Identifier/NumberIdentifierInterface.h"

class NumberIdentifier_V1 {
private:
    cv::dnn::Net _net;
    std::vector<cv::Mat> _templateFigures;//模板图像
public:
    explicit NumberIdentifier_V1(const char* model);
    NumberIdentifier_V1(const NumberIdentifier_V1&) = delete;
    NumberIdentifier_V1(NumberIdentifier_V1&&) = delete;

    bool Identify(const cv::Mat& imgGray, ArmorPlate& armor);
};

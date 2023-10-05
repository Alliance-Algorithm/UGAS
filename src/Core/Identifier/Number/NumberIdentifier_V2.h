#pragma once
/*
Creation Date: 2023/06/08
Latest Update: 2023/06/08
Developer(s): 22-Qzh 22-Ljc
Reference: ChenJun MLP Classifier
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 数字识别V2版本
- 综合两个网络的结果，使用统计结果区分大小装甲板
*/

#include <opencv2/opencv.hpp>

#include "Core/Identifier/Armor/ArmorStruct.h"
#include "Core/Identifier/NumberIdentifierInterface.h"
#include "Util/Debug/DebugCanvas.h"

class NumberIdentifier_V2 {
public:
    NumberIdentifier_V2(const char* modelCNN, const char* modelMLP) :
        _netCNN(cv::dnn::readNetFromTensorflow(modelCNN)),
        _netMLP(cv::dnn::readNetFromONNX(modelMLP)) {
    }
    NumberIdentifier_V2(const NumberIdentifier_V2&) = delete;
    NumberIdentifier_V2(NumberIdentifier_V2&&) = delete;

    bool Identify(const cv::Mat& imgGray, ArmorPlate& armor) {
        cv::Mat imgNumber, imgInput, imgWarped;
        //cv::Mat imgWarped = getWarpedImage(imgGray, armor, { 36, 36 }, { 45, 15.75 });
        double thre;

        if (!armor.is_large_armor)
            imgWarped = getWarpedImage(imgGray, armor, { 20, 28 }, { 32, 12 });
        else
            imgWarped = getWarpedImage(imgGray, armor, { 20, 28 }, { 54, 12 });

        double maxVal = 0;
        cv::minMaxLoc(imgWarped, nullptr, &maxVal);
        if (maxVal < 120) {
            thre = cv::threshold(imgWarped, imgNumber, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
            armor.id = getArmorIDFromMLP(imgNumber);

            if (armor.id == ArmorID::Base || armor.id == ArmorID::Engineer || armor.id == ArmorID::Sentry)
                armor.id = ArmorID::Unknown;
            if (armor.id != ArmorID::Unknown) {
                if (!armor.is_large_armor) {
                    switch (armor.id) {
                    case ArmorID::Hero:
                        armor.id = ArmorID::Unknown;
                    }
                }
                else {
                    switch (armor.id) {
                    case ArmorID::Engineer: case ArmorID::Sentry: case ArmorID::Outpost:
                        armor.id = ArmorID::Unknown;
                    }
                }
            }

            if constexpr (debugCanvas.armorNum) {
                //imgNumber *= 255.0;
                // 绘制二值化图像
                cv::cvtColor(imgNumber, imgNumber, cv::COLOR_GRAY2BGR);
                cv::Point drawPos = static_cast<cv::Point>(armor.center());
                //cv::Rect drawRect = cv::Rect(drawPos.x - 18, drawPos.y - 54, 36, 36);
                cv::Rect drawRect = cv::Rect(drawPos.x - 10, drawPos.y - 42, 20, 28);
                const auto& tl = drawRect.tl();
                if (tl.x >= 0 && tl.y >= 0) imgNumber.copyTo(debugCanvas.armorNum.GetMat()(drawRect));
                // 绘制识别数字
                cv::putText(debugCanvas.armorNum.GetMat(), std::to_string((int)armor.id), cv::Point(drawPos.x - 12, drawPos.y + 13), 1, 2.5, COLOR_GREEN, 3);
                // 绘制二值化阈值
                cv::putText(debugCanvas.armorNum.GetMat(), std::to_string(static_cast<int>(thre + 0.5)), armor.points[2], 1, 1.5, COLOR_GREEN, 1.5);
            }
        }
        else armor.id = ArmorID::Unknown;

        return armor.id != ArmorID::Unknown;
    }

private:
    cv::Mat getWarpedImage(const cv::Mat& imgGray, ArmorPlate& armor, const cv::Size2f& targetSize, const cv::Size2f& lightbarRect) {
        static std::vector<cv::Point2f> dst = { {0, 0}, {0, 0}, {0, 0}, {0, 0} };
        cv::Size2f delta = (targetSize - lightbarRect) / 2.0f;
        float x1 = delta.width, y1 = delta.height;
        float x2 = targetSize.width - delta.width, y2 = targetSize.height - delta.height;
        dst[0] = { x1, y1 };
        dst[1] = { x1, y2 };
        dst[2] = { x2, y2 };
        dst[3] = { x2, y1 };

        cv::Mat imgWarped, imgNumber, imgInput;
        cv::Mat M = getPerspectiveTransform(armor.points, dst);
        cv::warpPerspective(imgGray, imgWarped, M, targetSize);
        return imgWarped;
    }

    ArmorID getArmorIDFromMLP(const cv::Mat& imgNumber) {
        // Create blob from image
        cv::Mat blob;
        cv::dnn::blobFromImage(imgNumber / 255.0, blob);

        // Set the input blob for the neural network
        _netMLP.setInput(blob);
        // Forward pass the image blob through the model
        cv::Mat outputs = _netMLP.forward();

        // Do softmax
        float max_prob = *std::max_element(outputs.begin<float>(), outputs.end<float>());
        cv::Mat softmax_prob;
        cv::exp(outputs - max_prob, softmax_prob);
        float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
        softmax_prob /= sum;

        double confidence;
        cv::Point class_id_point;
        cv::minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
        switch (class_id_point.x) {
        case 0:
            return ArmorID::Hero;
        case 1:
            return ArmorID::Engineer;
        case 2:
            return ArmorID::InfantryIII;
        case 3:
            return ArmorID::InfantryIV;
        case 4:
            return ArmorID::InfantryV;
        case 5:
            return ArmorID::Outpost;
        case 6:
            return ArmorID::Sentry;
        case 7:
            return ArmorID::Base;
        default:
            return ArmorID::Unknown;
        }
    }

    cv::dnn::Net _netCNN; //输入36x36二值化图
    cv::dnn::Net _netMLP; //输入20x28二值化图
};

#pragma once

#include <vector>

#include "Core/Identifier/Armor/ArmorStruct.h"

class StandaloneArmorIdentifierInterface {
public:
    virtual ~StandaloneArmorIdentifierInterface() = default;

    virtual std::vector<ArmorPlate> Identify(const cv::Mat& img) = 0;
};
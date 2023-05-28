#pragma once
/*
Creation Date: 2022/11/20
Latest Update: 2022/11/20
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 就是个充数的，有空慢慢搞数字识别，这玩意只会返回NUM_DEFAULT
*/

#include "Core/Identifier/NumberIdentifierInterface.h"

class ArmorPlate;

class NullNumberIdentifier : public NumberIdentifierInterface {
public:
    void init(void*) {}
    short Identify(const cv::Mat& imgGray, const ArmorPlate& region) {
        return static_cast<short>(5);
    }
};

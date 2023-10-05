#pragma once
/*
Creation Date: 2023/08/09
Latest Update: 2023/08/09
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- Target的接口
*/

#include "Core/Transformer/Tree.h"

class TargetInterface {
public:
    virtual ~TargetInterface() = default;
    [[nodiscard]] virtual GimbalGyro::Position Predict(double sec) const = 0;
};
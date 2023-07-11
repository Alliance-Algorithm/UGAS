#pragma once
/*
Creation Date: 2023/5/22
Latest Update: 2023/5/22
Developer(s): 21-WZY 21-WTR
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- interface for buff PnP solver
*/

#include <optional>

#include <opencv2/opencv.hpp>

#include "Control/Gimbal/Gimbal.h"
#include "Core/Identifier/Buff/BuffStruct.h"


class BuffPnPSolverInterface {
public:
	virtual ~BuffPnPSolverInterface() = default;

	virtual std::optional<cv::Point3f> Solve(const Buff5Point& buff5Point) = 0;
};

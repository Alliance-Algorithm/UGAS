#pragma once
/*
Creation Date: 2023/08/02
Latest Update: 2023/08/02
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 参数：上供弹新全向轮
- 警告：禁止直接在外部引用此头文件，切换不同车辆参数请编辑parameters.cpp
*/

namespace parameters {
    const bool RotateCameraImage = true;

    const cv::Mat CameraMatrix(3, 3, CV_64F, camera_calibration_data::CameraMatrixData6);
    const cv::Mat CameraDistCoeffs(1, 5, CV_64F, camera_calibration_data::CameraDistCoeffsData6);

    const std::chrono::duration<float, std::micro> CameraExposureTime = std::chrono::microseconds(2000);
    const float CameraGain = MaxGainCS016, CameraDigitalShift = 0.0f;

    const double MaxArmorDistance = 15.0;

    const double AverageBulletSpeed30 = 26.0;
    const double AverageBulletSpeed18 = 15.6;
    const double AverageBulletSpeed15 = 12.2;

    const double StaticYawOffset = 0.4 / 180.0 * parameters::Pi;
    const double StaticPitchOffset = 2.4 / 180.0 * parameters::Pi;
}
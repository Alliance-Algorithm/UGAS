#pragma once
/*
Creation Date: 2022/10/11
Latest Update: 2023/05/09
Developer(s): 21-THY 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 声明并初始化所有参数
- 根据 "DebugSettings.h" 创建 Debug 拖动条
- 提供调试性函数
- 此文件未来可能被废弃或重组
*/

#include <cstdint>

#include <vector>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>


enum class GimbalType {
    Infantry,
    Uav,
    Sentry
};

enum class ArmorColor : uint8_t {
    Blue = 0, Red = 1
};


namespace parameters {
    /******** Universal Parameter Area ********/

    extern const ArmorColor DefaultEnemyColor;

    // parameters for hik camera: CS016
    inline constexpr float MaxGainCS016 = 16.9807f;
    inline constexpr float MaxDigitalShiftCS016 = 5.9993f;

    // identifier parameters
    inline constexpr float BlueLightBarHue = 228.0f;
    inline constexpr float RedLightBarHue = 11.0f;

    // armor matching parameters
    extern const double maxArmorLightRatio, maxdAngle, maxMalposition, maxLightDy, bigArmorDis;

    // pnp parameters
    // tl -> bl -> br -> tr
    extern const std::vector<cv::Point3d> NormalArmorObjectPoints, LargeArmorObjectPoints;

    // trajectory parameters
    extern const double DefaultBulletSpeed;
    inline constexpr double G = 9.8;

    inline constexpr double Pi = 3.14159265358979323846;


    /******** Specialized Parameter Area ********/

    // type of gimbal
    extern const GimbalType GimbalType;

    extern const Eigen::Translation3d TranslationGimbalToCamera, TranslationGimbalToMuzzle;

    // flag whether to rotate the camera image 180 degrees.
    extern const bool RotateCameraImage;

    // camera calibration matrix
    extern const cv::Mat CameraMatrix, CameraDistCoeffs;

    // camera initialization parameters
    extern const std::chrono::duration<float, std::micro> CameraExposureTime;
    extern const float CameraGain, CameraDigitalShift;

    // will ignore armor which distance farther than this
    extern const double MaxArmorDistance;

    // will be used by class cboard_xx to mapping preset bullet speeds and actual bullet speeds.
    // average bullet speed when max bullet speed set to 30/18/15.
    extern const double AverageBulletSpeed30, AverageBulletSpeed18, AverageBulletSpeed15;

    // offset applied to the trajectory result.
    extern const double StaticYawOffset, StaticPitchOffset;
};

/* Colors */
#define COLOR_RED                  cv::Scalar(0    ,0        ,255)
#define COLOR_PINK                 cv::Scalar(196    ,196    ,255)
#define COLOR_DARKRED              cv::Scalar(0    ,0        ,128)

#define COLOR_BLUE                 cv::Scalar(255    ,0        ,0)
#define COLOR_LIGHTBLUE            cv::Scalar(255    ,196    ,196)
#define COLOR_DARKBLUE             cv::Scalar(0    ,0        ,128)

#define COLOR_GREEN                cv::Scalar(0    ,255    ,0)
#define COLOR_LIGHTGREEN           cv::Scalar(196    ,255    ,196)
#define COLOR_DARKGREEN            cv::Scalar(0    ,128    ,0)

#define COLOR_BLACK                cv::Scalar(0    ,0        ,0)
#define COLOR_DARKGRAY             cv::Scalar(88    ,88        ,88)
#define COLOR_LIGHTGRAY            cv::Scalar(192    ,192    ,192)
#define COLOR_WHITE                cv::Scalar(255    ,255    ,255)

#define COLOR_PURPLE               cv::Scalar(255    ,0        ,255)
#define COLOR_YELLOW               cv::Scalar(0    ,255    ,255)
#define COLOR_VIOLENT              cv::Scalar(128    ,0        ,128)
#define COLOR_MINTGREEN            cv::Scalar(204    ,255    ,0)
#define COLOR_BROWN                cv::Scalar(0    ,63        ,125)
#define COLOR_MALACHITEGREEN       cv::Scalar(128    ,128    ,0)
#define COLOR_EARTHYYELLOW         cv::Scalar(0    ,128    ,128)
#define COLOR_ORANGE               cv::Scalar(0    ,128    ,255)
#define COLOR_LIME                 cv::Scalar(255    ,255    ,0)

#pragma once

#include <cstdint>

#include <vector>

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

class ArmorDetector final {
public:
    enum class ArmorColor : uint8_t { BLUE = 0, RED = 1 };
    enum class ArmorId : uint16_t {
        UNKNOWN      = 0,
        HERO         = 1,
        ENGINEER     = 2,
        INFANTRY_III = 3,
        INFANTRY_IV  = 4,
        INFANTRY_V   = 5,
        SENTRY       = 6,
        LARGE_III    = 7,
        LARGE_IV     = 8,
        LARGE_V      = 9,
        OUTPOST      = 10,
        BASE         = 11,
        // Mar 27 适配数字神经网络识别结果
    };

    struct ArmorPlate {
        ArmorId id;
        Eigen::Vector3d position;
        Eigen::Quaterniond rotation;
        ArmorPlate(ArmorId id, const Eigen::Vector3d& position, const Eigen::Quaterniond& rotation)
            : id(id)
            , position(position)
            , rotation(rotation) {}
    };

    ArmorDetector();

    ArmorDetector(const ArmorDetector&)            = delete;
    ArmorDetector& operator=(const ArmorDetector&) = delete;

    ~ArmorDetector();

    [[nodiscard]] std::vector<ArmorPlate> detect(const cv::Mat& img, ArmorColor target_color);

private:
    class Impl;
    Impl* impl_;
};
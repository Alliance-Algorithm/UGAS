#pragma once

#include <cstdint>

#include <vector>

#include <opencv2/core/mat.hpp>

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
        OUTPOST      = 7,
        BASE         = 8,
    };

    // struct LightBar {
    //     cv::Point2f top, bottom;
    //     float angle;
        
    //     LightBar(cv::Point2f top, cv::Point2f bottom, float angle)
    //         : top(top)
    //         , bottom(bottom)
    //         , angle(angle) {}
    // };
    // struct ArmorPlate {
    //     ArmorPlate(const LightBar& left, const LightBar& right, ArmorId id, bool is_large_armor)
    //         : id(id)
    //         , is_large_armor(is_large_armor) {
    //         // 老代码传承的顺序：U字型（和PNP参数表示顺序同步）
    //         // 跟opencv使用的顺序不同，后面要改掉（三个地方：装甲板参数、数字识别、PNP部分)
    //         points.push_back(left.top);
    //         points.push_back(left.bottom);
    //         points.push_back(right.bottom);
    //         points.push_back(right.top);
    //     }

    //     std::vector<cv::Point2f> points;
    //     ArmorId id;
    //     bool is_large_armor;
    // };

    struct ArmorPlate {
        ArmorId id;
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
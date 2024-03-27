#include "armor_detector.hpp"

#include <cstdint>

#include <opencv2/core/mat.hpp>
#include <stdexcept>
#include <variant>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

// 数字神经网络接口
class NumberIdentifyInterface {
public:
    virtual ~NumberIdentifyInterface()                     = default;
    virtual std::tuple<int, double> Identify(cv::Mat& img) = 0;
};

class NumberIdentify : public NumberIdentifyInterface {
public:
    explicit NumberIdentify(std::string modelPath);
    std::tuple<int, double> Identify(cv::Mat& img);

private:
    cv::dnn::Net _net;
    void _softmax(cv::Mat& modelInferResult);
    cv::Mat _BlobImage(cv::Mat& img);
};

/**
 * @brief Construct a new Number Identify:: Number Identify object
 *
 * @param modelPath 支持onnx与pb模型
 */
NumberIdentify::NumberIdentify(std::string modelPath) {
    if (modelPath.find(".onnx") != std::variant_npos) {
        _net = cv::dnn::readNetFromONNX(modelPath);
    } else if (modelPath.find(".pb") != std::variant_npos) {
        _net = cv::dnn::readNetFromTensorflow(modelPath);
    } else {
        throw std::runtime_error("Error model type!");
    }
    if (_net.empty())
        throw std::runtime_error("Cannot open model file!");
}

/**
 * @brief 输出识别结果与置信度，识别结果可转ArmorDetector::ArmorId枚举类
 *
 * @param img
 * @return std::tuple<int, double>
 */
std::tuple<int, double> NumberIdentify::Identify(cv::Mat& img) {
    cv::Mat inputImage = _BlobImage(img);
    cv::Mat blobImage  = cv::dnn::blobFromImage(inputImage, 1.0, cv::Size(36, 36), false, false);
    _net.setInput(blobImage);
    cv::Mat pred = _net.forward();
    cv::normalize(pred, pred, 1, 10, cv::NORM_MINMAX);
    _softmax(pred);
    double maxValue;
    cv::Point maxLoc;
    minMaxLoc(pred, NULL, &maxValue, NULL, &maxLoc);
    std::tuple<int, double> resultMsg(maxLoc.x, maxValue);

    return resultMsg;
}

cv::Mat NumberIdentify::_BlobImage(cv::Mat& img) {
    cv::resize(img, img, cv::Size(36, 36));
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(img, img, cv::Size(5, 5), 0, 0);
    cv::threshold(img, img, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    return img;
}

inline void NumberIdentify::_softmax(cv::Mat& modelInferResult) {
    cv::Mat expMat;
    cv::exp(modelInferResult, expMat);

    double sumExp = cv::sum(expMat)[0];

    modelInferResult = expMat / sumExp;
}

class ColorIdentifier {
public:
    ColorIdentifier(float hue360) {
        if (hue360 > 180) // (temp) blue
            _cc1 = 0, _cc2 = 1, _cc3 = 2;
        else              // (temp) red
            _cc1 = 2, _cc2 = 1, _cc3 = 0;
        min_hue_        = hue360 - 5;
        max_hue_        = hue360 + 5;
        min_hue_uchar_  = min_hue_ / 360 * 255;
        max_hue_uchar_  = max_hue_ / 360 * 255;
        confidence_map_ = generate_map();
    }

    enum class Confidence : uint8_t {
        NotCredible                      = 0,
        CredibleZeroChannelOverexposure  = 255,
        CredibleOneChannelOverexposure   = 191,
        CredibleTwoChannelOverexposure   = 127,
        CredibleThreeChannelOverexposure = 63
    };

    Confidence identify(const uint8_t* color) const {
        // 部分代码来自
        // https://github.com/egonSchiele/OpenCV/blob/master/modules/imgproc/src/color.cpp
        constexpr int bidx      = 0; //, scn = 3;
        constexpr int hsv_shift = 12;

        static constexpr int div_table[] = {
            0,     1044480, 522240, 348160, 261120, 208896, 174080, 149211, 130560, 116053, 104448,
            94953, 87040,   80345,  74606,  69632,  65280,  61440,  58027,  54973,  52224,  49737,
            47476, 45412,   43520,  41779,  40172,  38684,  37303,  36017,  34816,  33693,  32640,
            31651, 30720,   29842,  29013,  28229,  27486,  26782,  26112,  25475,  24869,  24290,
            23738, 23211,   22706,  22223,  21760,  21316,  20890,  20480,  20086,  19707,  19342,
            18991, 18651,   18324,  18008,  17703,  17408,  17123,  16846,  16579,  16320,  16069,
            15825, 15589,   15360,  15137,  14921,  14711,  14507,  14308,  14115,  13926,  13743,
            13565, 13391,   13221,  13056,  12895,  12738,  12584,  12434,  12288,  12145,  12006,
            11869, 11736,   11605,  11478,  11353,  11231,  11111,  10995,  10880,  10768,  10658,
            10550, 10445,   10341,  10240,  10141,  10043,  9947,   9854,   9761,   9671,   9582,
            9495,  9410,    9326,   9243,   9162,   9082,   9004,   8927,   8852,   8777,   8704,
            8632,  8561,    8492,   8423,   8356,   8290,   8224,   8160,   8097,   8034,   7973,
            7913,  7853,    7795,   7737,   7680,   7624,   7569,   7514,   7461,   7408,   7355,
            7304,  7253,    7203,   7154,   7105,   7057,   7010,   6963,   6917,   6872,   6827,
            6782,  6739,    6695,   6653,   6611,   6569,   6528,   6487,   6447,   6408,   6369,
            6330,  6292,    6254,   6217,   6180,   6144,   6108,   6073,   6037,   6003,   5968,
            5935,  5901,    5868,   5835,   5803,   5771,   5739,   5708,   5677,   5646,   5615,
            5585,  5556,    5526,   5497,   5468,   5440,   5412,   5384,   5356,   5329,   5302,
            5275,  5249,    5222,   5196,   5171,   5145,   5120,   5095,   5070,   5046,   5022,
            4998,  4974,    4950,   4927,   4904,   4881,   4858,   4836,   4813,   4791,   4769,
            4748,  4726,    4705,   4684,   4663,   4642,   4622,   4601,   4581,   4561,   4541,
            4522,  4502,    4483,   4464,   4445,   4426,   4407,   4389,   4370,   4352,   4334,
            4316,  4298,    4281,   4263,   4246,   4229,   4212,   4195,   4178,   4161,   4145,
            4128,  4112,    4096};
        constexpr int hr = true ? 255 : 180, hscale = hr == 180 ? 15 : 21;

        int b = color[bidx], g = color[1], r = color[bidx ^ 2];
        int h, s, v                          = b;

        v = v > g ? v : g;
        v = v > r ? v : r;
        if (v > min_value_uchar_) {
            if (color[_cc1] >= 250) {
                return confidence_map_.at<Confidence>(color[_cc3], color[_cc2]);
            } else {
                int vmin = b;
                vmin     = vmin < g ? vmin : g;
                vmin     = vmin < r ? vmin : r;
                int diff = v - vmin;

                int vr = v == r ? -1 : 0;
                int vg = v == g ? -1 : 0;

                s = diff * div_table[v] >> hsv_shift;
                if (s > min_saturation_uchar_) {
                    h = (vr & (g - b))
                      + (~vr & ((vg & (b - r + 2 * diff)) + ((~vg) & (r - g + 4 * diff))));
                    h = (h * div_table[diff] * hscale + (1 << (hsv_shift + 6))) >> (7 + hsv_shift);
                    h += h < 0 ? hr : 0;
                    if (min_hue_uchar_ < h && h < max_hue_uchar_)
                        return Confidence::CredibleZeroChannelOverexposure;
                }
            }
        }

        return Confidence::NotCredible;
    }

private:
    // 过曝顺序
    int _cc1, _cc2, _cc3;

    static constexpr float min_saturation_ = 0.8, min_value_ = 0.5;
    static constexpr uint8_t min_saturation_uchar_ = min_saturation_ * 255,
                             min_value_uchar_      = min_value_ * 255;
    float min_hue_, max_hue_;
    uint8_t min_hue_uchar_, max_hue_uchar_;

    cv::Mat confidence_map_;

    cv::Mat generate_map() {
        // 暂时只能生成蓝色和红色
        constexpr int renderSize = 256;

        cv::Mat imgBGR, imgHSV, imgMap;
        imgBGR.create(renderSize, renderSize, CV_32FC3);
        imgMap.create(renderSize, renderSize, CV_8UC1);

        for (int i = 0; i < renderSize; ++i) {
            for (int j = 0; j < renderSize; ++j) {
                cv::Vec3f color;
                color[_cc1]                = 1.0f;
                color[_cc2]                = static_cast<float>(i) / renderSize;
                color[_cc3]                = static_cast<float>(j) / renderSize;
                imgBGR.at<cv::Vec3f>(j, i) = color;
                imgMap.at<uint8_t>(j, i)   = static_cast<uint8_t>(Confidence::NotCredible);
            }
        }
        cv::cvtColor(imgBGR, imgHSV, cv::COLOR_BGR2HSV);
        int cornerPointX = 0;
        float maxSlope   = 0;
        int maxFitY[renderSize];
        for (int i = 0; i < renderSize; ++i)
            maxFitY[i] = -1;
        for (int i = 0; i < renderSize; ++i) {     // x
            for (int j = 0; j < renderSize; ++j) { // y
                auto& hsv = imgHSV.at<cv::Vec3f>(j, i);
                if (min_hue_ < hsv[0] && hsv[0] < max_hue_ && hsv[1] > min_saturation_
                    && hsv[2] > min_value_) {
                    imgMap.at<uint8_t>(j, i) =
                        static_cast<uint8_t>(Confidence::CredibleZeroChannelOverexposure);
                    maxFitY[i] = j;
                    if (i > 0) {
                        float slope = static_cast<float>(j) / i;
                        if (slope > maxSlope) {
                            maxSlope     = slope;
                            cornerPointX = i;
                        }
                    }
                } else
                    imgBGR.at<cv::Vec3f>(j, i) = {1.0f, 1.0f, 1.0f};
            }
        }
        for (int i = 0; i < renderSize; ++i) {     // x
            for (int j = 0; j < maxFitY[i]; ++j) { // y
                auto& pixel = imgMap.at<uint8_t>(j, i);
                if (pixel != static_cast<uint8_t>(Confidence::CredibleZeroChannelOverexposure))
                    pixel = static_cast<uint8_t>(Confidence::CredibleOneChannelOverexposure);
            }
        }
        for (int i = cornerPointX; i < renderSize; ++i) {
            int maxY = maxSlope * i + 0.5;
            for (int j = 0; j <= maxY; ++j) {
                auto& pixel = imgMap.at<uint8_t>(j, i);
                if (pixel != static_cast<uint8_t>(Confidence::CredibleZeroChannelOverexposure))
                    pixel = static_cast<uint8_t>(Confidence::CredibleOneChannelOverexposure);
            }
        }
        for (int j = 0; j < renderSize; ++j) {
            if (j == renderSize - 1)
                imgMap.at<uint8_t>(j, renderSize - 1) =
                    static_cast<uint8_t>(Confidence::CredibleThreeChannelOverexposure);
            else
                imgMap.at<uint8_t>(j, renderSize - 1) =
                    static_cast<uint8_t>(Confidence::CredibleTwoChannelOverexposure);
        }

        if constexpr (renderSize == 256) {
            return imgMap;
        } else {
            auto& imgDisplay = false ? imgBGR : imgMap;
            cv::flip(imgDisplay, imgDisplay, 0);
            cv::imshow("debug", imgDisplay);
            cv::waitKey(-1);
            throw(std::runtime_error("render size != 256, enable debugging..."));
        }
    }
};

class ArmorDetector::Impl {
public:
    Impl()
        : _blueIdentifier(228.0f)
        , _redIdentifier(11.0f)
        // TODO： 模型路径优化
        , _identify(
              NumberIdentify("/workspaces/RMCS/rmcs_ws/src/ugas/models/armoridentify_v3.onnx")) {}

    inline std::vector<ArmorPlate> detect(const cv::Mat& img, ArmorColor target_color) {
        cv::Mat threshold_img, gray_img;
        cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);
        cv::threshold(gray_img, threshold_img, 150, 255, cv::THRESH_BINARY);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(threshold_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

        auto lightbars = solve_lightbars(img, contours, target_color);
        auto matched_lightbars = match_lightbars(lightbars, img); // 其中包括数字神经网络识别
        auto result = solve_pnp(matched_lightbars);

        return result;
    }

private:
    struct LightBar {
        LightBar(cv::Point2f top, cv::Point2f bottom, float angle)
            : top(top)
            , bottom(bottom)
            , angle(angle) {}

        cv::Point2f top, bottom;
        float angle;
    };

    struct MatchedLightBar {
        MatchedLightBar(const LightBar& left, const LightBar& right) {
            points[0] = left.top;
            points[1] = left.bottom;
            points[2] = right.bottom;
            points[3] = right.top;

            isLarge = false;
        }

        cv::Point2f top_left() const { return points[0]; }
        cv::Point2f bottom_left() const { return points[1]; }
        cv::Point2f bottom_right() const { return points[2]; }
        cv::Point2f top_right() const { return points[3]; }

        cv::Point2f points[4];
        bool isLarge;                                             // 大装甲板标志
    };

    inline std::vector<LightBar> solve_lightbars(
        const cv::Mat& img, const std::vector<std::vector<cv::Point>>& contours,
        ArmorColor targetColor) {

        std::vector<LightBar> lightbars;
        for (const auto& contour : contours) {

            auto&& contourSize = contour.size();
            if (contourSize >= 5) {
                float scoreMap[256];
                float confidence                                                        = 0.0f;
                scoreMap[static_cast<size_t>(ColorIdentifier::Confidence::NotCredible)] = 0.0f;
                scoreMap[static_cast<size_t>(
                    ColorIdentifier::Confidence::CredibleOneChannelOverexposure)] =
                    1.0f / (float)contourSize;
                scoreMap[static_cast<size_t>(
                    ColorIdentifier::Confidence::CredibleTwoChannelOverexposure)] =
                    0.5f / (float)contourSize;
                scoreMap[static_cast<size_t>(
                    ColorIdentifier::Confidence::CredibleThreeChannelOverexposure)] =
                    0.2f / (float)contourSize;

                auto& colorIdentifier =
                    targetColor == ArmorColor::BLUE ? _blueIdentifier : _redIdentifier;
                int maxPointY = 0;
                for (const auto& point : contour) {
                    maxPointY = std::max(maxPointY, point.y);
                    auto c    = reinterpret_cast<const uint8_t*>(&img.at<cv::Vec3b>(point));
                    confidence += scoreMap[static_cast<size_t>(colorIdentifier.identify(c))];
                }
                if (img.rows == maxPointY + 1)
                    confidence = 0;

                if (confidence > 0.45f) {
                    constexpr float angle_range = 30;
                    auto box                    = cv::minAreaRect(contour);

                    cv::Point2f corner[4];
                    box.points(corner);
                    auto diff = box.size.width - box.size.height;
                    if (diff > 0) {
                        float angle = fmodf(box.angle + 360, 360);
                        if (90 - angle_range < angle && angle < 90 + angle_range) {
                            lightbars.emplace_back(
                                (corner[0] + corner[1]) / 2, (corner[2] + corner[3]) / 2, 0);
                        } else if (270 - angle_range < angle && angle < 270 + angle_range) {
                            lightbars.emplace_back(
                                (corner[2] + corner[3]) / 2, (corner[0] + corner[1]) / 2, 0);
                        }
                    } else if (diff < 0) {
                        float angle = fmodf(box.angle + 360 + 90, 360);
                        if (90 - angle_range < angle && angle < 90 + angle_range) {
                            lightbars.emplace_back(
                                (corner[1] + corner[2]) / 2, (corner[0] + corner[3]) / 2, 0);
                        } else if (270 - angle_range < angle && angle < 270 + angle_range) {
                            lightbars.emplace_back(
                                (corner[0] + corner[3]) / 2, (corner[1] + corner[2]) / 2, 0);
                        }
                    }
                }
            }
        }
        return lightbars;
    }

    inline std::vector<MatchedLightBar>
        match_lightbars(std::vector<LightBar>& lightbars, const cv::Mat& img) {
        std::sort(lightbars.begin(), lightbars.end(), [](LightBar& a, LightBar& b) {
            return a.top.x < b.top.x;
        });

        std::vector<MatchedLightBar> matched;

        constexpr double maxArmorLightRatio = 1.5, maxdAngle = 9.5, maxMalposition = 0.7,
                         maxLightDy = 0.9, bigArmorDis = 5.0;
        size_t&& lightBarsSize = lightbars.size();

        for (size_t i = 0; i < lightBarsSize; ++i) {
            float Isize         = cv::norm(lightbars[i].top - lightbars[i].bottom);
            cv::Point2f Icenter = (lightbars[i].top + lightbars[i].bottom) / 2;
            for (size_t j = i + 1; j < lightBarsSize; ++j) { // 一些筛选条件

                float Jsize = cv::norm(lightbars[j].top - lightbars[j].bottom);
                if (fmax(Isize, Jsize) / fmin(Isize, Jsize) > maxArmorLightRatio)
                    continue;
                if (fabs(lightbars[i].angle - lightbars[j].angle) > maxdAngle)
                    continue;
                if (malposition(lightbars[i], lightbars[j]) > maxMalposition)
                    continue;
                cv::Point2f Jcenter = (lightbars[j].top + lightbars[j].bottom) / 2;
                if (fabs(Icenter.y - Jcenter.y) * 2 / (Isize + Jsize) > maxLightDy)
                    continue;
                float lightBarDis = cv::norm(Icenter - Jcenter) * 2 / (Isize + Jsize);
                if (lightBarDis > bigArmorDis)
                    continue;

                matched.emplace_back(lightbars[i], lightbars[j]);
            }
        }

        std::vector<MatchedLightBar> flitered; // 数字识别过滤后的装甲板容器

        for (auto& sample : matched) {
            cv::Mat roi;
            perspective(img, sample, roi);
            auto res        = _identify.Identify(roi);
            auto code       = std::get<0>(res);
            auto confidence = std::get<1>(res);

            if (confidence < 0.5) {
                continue;
            }

            switch ((ArmorId)code) {
            case ArmorId::LARGE_III:
            case ArmorId::LARGE_IV:
            case ArmorId::LARGE_V: sample.isLarge = true; break;
            default: break;
            }

            flitered.push_back(sample);
        }

        return flitered;
    }

    inline std::vector<ArmorPlate>
        solve_pnp(const std::vector<MatchedLightBar>& matched_lightbars) {

        constexpr double NormalArmorWidth = 134, NormalArmorHeight = 56, LargerArmorWidth = 230,
                         LargerArmorHeight                     = 56;
        const std::vector<cv::Point3d> NormalArmorObjectPoints = {
            cv::Point3d(-0.5 * NormalArmorWidth, 0.5 * NormalArmorHeight, 0.0f),
            cv::Point3d(-0.5 * NormalArmorWidth, -0.5 * NormalArmorHeight, 0.0f),
            cv::Point3d(0.5 * NormalArmorWidth, -0.5 * NormalArmorHeight, 0.0f),
            cv::Point3d(0.5 * NormalArmorWidth, 0.5 * NormalArmorHeight, 0.0f)};
        const std::vector<cv::Point3d> LargeArmorObjectPoints = {
            cv::Point3d(-0.5 * LargerArmorWidth, 0.5 * LargerArmorHeight, 0.0f),
            cv::Point3d(-0.5 * LargerArmorWidth, -0.5 * LargerArmorHeight, 0.0f),
            cv::Point3d(0.5 * LargerArmorWidth, -0.5 * LargerArmorHeight, 0.0f),
            cv::Point3d(0.5 * LargerArmorWidth, 0.5 * LargerArmorHeight, 0.0f)};

        double CameraMatrixData[3][3] = {
            {1.722231837421459e+03,                     0, 7.013056440882832e+02},
            {                    0, 1.724876404292754e+03, 5.645821718351237e+02},
            {                    0,                     0,                     1}
        };
        double CameraDistCoeffsData[5] = {
            -0.064232403853946, -0.087667493884102, 0, 0, 0.792381808294582};
        const cv::Mat CameraMatrix(3, 3, CV_64F, CameraMatrixData);
        const cv::Mat CameraDistCoeffs(1, 5, CV_64F, CameraDistCoeffsData);

        cv::Mat rvec, tvec;
        std::vector<ArmorPlate> armors;

        for (auto& matched : matched_lightbars) {

            std::vector<cv::Point2f> image_points;
            for (auto& point : matched.points)
                image_points.push_back(point);

            auto objectPoints = NormalArmorObjectPoints;

            if (matched.isLarge) {
                objectPoints = LargeArmorObjectPoints;
                // Mar 27 fix 装甲板大小 Feng
            }

            if (cv::solvePnP(
                    objectPoints, image_points, CameraMatrix, CameraDistCoeffs, rvec, tvec, false,
                    cv::SOLVEPNP_IPPE)) {

                Eigen::Vector3d position = {
                    tvec.at<double>(2), -tvec.at<double>(0), -tvec.at<double>(1)};
                position = position / 1000.0;
                if (position.norm() > 15.0)
                    continue;

                Eigen::Vector3d rvec_eigen = {
                    rvec.at<double>(2), -rvec.at<double>(0), -rvec.at<double>(1)};
                Eigen::Quaterniond rotation = Eigen::Quaterniond{
                    Eigen::AngleAxisd{rvec_eigen.norm(), rvec_eigen.normalized()}
                };

                armors.emplace_back(ArmorId::UNKNOWN, position, rotation);
            }
        }

        return armors;
    }

    inline static double malposition(const LightBar& left, const LightBar& right) {
        cv::Point2f axis = (left.top - left.bottom + right.top - right.bottom) / 2;
        cv::Point2f dis  = (left.top + left.bottom - right.top - right.bottom) / 2;
        return fabs(axis.dot(dis) / axis.cross(dis));
    };

    /**
     * @brief 透视矫正
     *
     * @param img org
     * @param match_lightbar 矫正特征点
     * @param roi dst
     */
    inline static void
        perspective(const cv::Mat& img, MatchedLightBar& match_lightbar, cv::Mat& roi) {
        auto leftHeight  = cv::norm(match_lightbar.top_left() - match_lightbar.bottom_left());
        auto rightHeight = cv::norm(match_lightbar.top_right() - match_lightbar.bottom_right());
        auto maxHeight   = std::max(leftHeight, rightHeight);

        auto upWidth   = cv::norm(match_lightbar.top_left() - match_lightbar.top_right());
        auto downWidth = cv::norm(match_lightbar.bottom_left() - match_lightbar.bottom_right());
        auto maxWidth  = std::max(upWidth, downWidth);

        cv::Point2f srcAffinePts[4] = {
            cv::Point2f(match_lightbar.top_left()), cv::Point2f(match_lightbar.top_right()),
            cv::Point2f(match_lightbar.bottom_right()), cv::Point2f(match_lightbar.bottom_left())};
        cv::Point2f dstAffinePts[4] = {
            cv::Point2f(0, 0), cv::Point2f(maxWidth, 0), cv::Point2f(maxWidth, maxHeight),
            cv::Point2f(0, maxHeight)};

        auto affineMat = cv::getPerspectiveTransform(srcAffinePts, dstAffinePts);
        cv::warpPerspective(img, roi, affineMat, cv::Point(maxWidth, maxHeight));
    }

    ColorIdentifier _blueIdentifier, _redIdentifier;
    NumberIdentify _identify;
};

ArmorDetector::ArmorDetector() { impl_ = new Impl{}; }

ArmorDetector::~ArmorDetector() { delete impl_; }

[[nodiscard]] std::vector<ArmorDetector::ArmorPlate>
    ArmorDetector::detect(const cv::Mat& img, ArmorColor target_color) {
    return impl_->detect(img, target_color);
}
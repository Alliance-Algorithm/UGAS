#include <cmath>
#include "Common/Color.h"
#include "ArmorIdentifier_V2.h"

inline std::vector<std::vector<cv::Point>> ArmorIdentifier_V2::_floodFindAreas(const cv::Mat& grayImg, int areaVal, int floodVal, bool borderEngulfment) {
    std::queue<cv::Point> emptyQueue;
    std::swap(emptyQueue, _dfsQueue);
    _disjointSet.Reset(grayImg.rows * grayImg.cols);
    _floodMap.Reset(grayImg.rows, grayImg.cols);
    for (int y = 0; y < grayImg.rows; ++y)
        for (int x = 0; x < grayImg.cols; ++x)
            if (grayImg.at<uchar>(y, x) >= areaVal) {
                _dfsQueue.push(cv::Point(x, y));
                _floodMap.At(y, x) = _disjointSet.Add(cv::Point(x, y));
            }
    int splitIndex = _disjointSet.Length;
    while (!_dfsQueue.empty()) {
        auto point = _dfsQueue.front();
        _dfsQueue.pop();
        auto p = _floodMap.At(point.y, point.x);
        bool isEdge = false;
        if (point.y > 0)
            _floodPixel(grayImg, floodVal, p, point.y - 1, point.x);
        else isEdge = true;
        if (point.y < grayImg.rows - 1)
            _floodPixel(grayImg, floodVal, p, point.y + 1, point.x);
        else isEdge = true;
        if (point.x > 0)
            _floodPixel(grayImg, floodVal, p, point.y, point.x - 1);
        else isEdge = true;
        if (point.x < grayImg.cols - 1)
            _floodPixel(grayImg, floodVal, p, point.y, point.x + 1);
        else isEdge = true;
        if (borderEngulfment && isEdge)
            _disjointSet.SetLevel(p, 1);
    }
    //return _disjointSet.GetGroups();
    return _disjointSet.GetGroups(splitIndex);
}

inline void ArmorIdentifier_V2::_floodPixel(const cv::Mat& grayImg, int floodVal, FastDisjointSet<cv::Point>::Node* source, int fy, int fx) {
    if (grayImg.at<uchar>(fy, fx) > floodVal) {
        if (_floodMap.At(fy, fx) == nullptr) {
            _floodMap.At(fy, fx) = _disjointSet.Add(cv::Point(fx, fy));
            _dfsQueue.push(cv::Point(fx, fy));
        }
        else {
            _disjointSet.Union(source, _floodMap.At(fy, fx));
        }
    }
}

inline bool ArmorIdentifier_V2::_solveToLightbar(const std::vector<cv::Point>& area) {
    if (area.size() > 10) {
        static const int angleRange = 30;
        auto box = cv::minAreaRect(area);
        auto width = box.size.width + 1;
        auto height = box.size.height + 1;
        auto fillRatio = area.size() / (width * height);
        if (0.4 < fillRatio && fillRatio < 1.5) {
            std::vector<cv::Point2f> contour(4);
            box.points(contour.data());
            auto diff = width - height;
            if (diff > 0) {     //旋转前，矩形横放
                float angle = fmod(box.angle + 360, 360);
                if (90 - angleRange < angle && angle < 90 + angleRange) {
                    _lightBarList.push_back(LightBar((contour[0] + contour[1]) / 2, (contour[2] + contour[3]) / 2, 0));
                    return true;
                }
                else if (270 - angleRange < angle && angle < 270 + angleRange) {
                    _lightBarList.push_back(LightBar((contour[2] + contour[3]) / 2, (contour[0] + contour[1]) / 2, 0));
                    return true;
                }
            }
            else if (diff < 0) {  //旋转前，矩形横放
                float angle = fmod(box.angle + 360 + 90, 360);
                if (90 - angleRange < angle && angle < 90 + angleRange) {
                    _lightBarList.push_back(LightBar((contour[1] + contour[2]) / 2, (contour[0] + contour[3]) / 2, 0));
                    return true;
                }
                else if (270 - angleRange < angle && angle < 270 + angleRange) {
                    _lightBarList.push_back(LightBar((contour[0] + contour[3]) / 2, (contour[1] + contour[2]) / 2, 0));
                    return true;
                }
            }
        }
    }    
    return false;
}

inline void ArmorIdentifier_V2::_matchArmorPlates(std::vector<ArmorPlate>& result) {
    result.clear();
    std::sort(_lightBarList.begin(), _lightBarList.end(),
        [&](LightBar& a, LightBar& b) {
            return a.top.x < b.top.x;
        });
    if (_lightBarList.size() >= 2)
        for (auto i = _lightBarList.begin() + 1; i != _lightBarList.end(); ++i) {
            result.push_back(ArmorPlate(*(i - 1), *i, 3));
        }
}

void ArmorIdentifier_V2::Identify(const Img& imgThre, const Img& imgGray, std::vector<ArmorPlate>& result) {
    auto areaList = _floodFindAreas(imgThre, 250, 120, true);
    _lightBarList.clear();
    for (auto i = areaList.begin(); i != areaList.end(); ++i)
        _solveToLightbar(*i);
    _matchArmorPlates(result);

#if DEBUG_LIGHTBAR == 1
    for (const auto& lightBar : _lightBarList) {
        line(debugImg, lightBar.top, lightBar.bottom, COLOR_BLUE, 5);
        circle(debugImg, lightBar.top, 2, COLOR_ORANGE, 2);
        circle(debugImg, lightBar.bottom, 2, COLOR_PINK, 2);
        auto angle = std::to_string(lightBar.angle); angle.resize(4);
        //TextFormat(angle).SetFontScale(0.5)
        //	.Draw(img, lightBar._bottom, COLOR_YELLOW, Direction::BOTTOM_RIGHT);
        putText(debugImg, angle, lightBar.bottom, 0, 0.5, COLOR_YELLOW);
    }
#endif
#if DEBUG_ARMOR == 1
    for (const auto& armorPlate : result) {
        const std::vector<cv::Point2f>& points = armorPlate.points;
        line(debugImg, points[0], points[1], COLOR_WHITE);
        line(debugImg, points[1], points[2], COLOR_LIGHTGRAY);
        line(debugImg, points[2], points[3], COLOR_DARKGRAY);
        line(debugImg, points[3], points[0], COLOR_RED);
        circle(debugImg, armorPlate.center(), 3, COLOR_GREEN, 2);
    }
#endif
}

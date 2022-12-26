#include "ArmorIdentifier_V2.h"
#include <cmath>
#include <Common/Color.h>
#include <Common/UniversalFunctions/UniversalFunctions.h>

inline std::vector<std::vector<cv::Point>> ArmorIdentifier_V2::_floodFindAreas(const cv::Mat& img, int areaVal, int floodVal, bool borderEngulfment) {
    std::queue<cv::Point> emptyQueue;
    std::swap(emptyQueue, _dfsQueue);
    _disjointSet.Reset(img.rows * img.cols);
    _floodMap.Reset(img.rows, img.cols);
    for (int y = 0; y < img.rows; ++y)
        for (int x = 0; x < img.cols; ++x)
            if (img.at<uchar>(y, x) == 255) {
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
            _floodPixel(img, floodVal, p, point.y - 1, point.x);
        else isEdge = true;
        if (point.y < img.rows - 1)
            _floodPixel(img, floodVal, p, point.y + 1, point.x);
        else isEdge = true;
        if (point.x > 0)
            _floodPixel(img, floodVal, p, point.y, point.x - 1);
        else isEdge = true;
        if (point.x < img.cols - 1)
            _floodPixel(img, floodVal, p, point.y, point.x + 1);
        else isEdge = true;
        if (/*img.at<uchar>(point) == 1 || */(borderEngulfment && isEdge))
            _disjointSet.SetLevel(p, 1);
    }
    //return _disjointSet.GetGroups();
    return _disjointSet.GetGroups(splitIndex);
}

inline void ArmorIdentifier_V2::_floodPixel(const cv::Mat& grayImg, int floodVal, FastDisjointSet<cv::Point>::Node* source, int fy, int fx) {
    if (grayImg.at<uchar>(fy, fx) > 0) {
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
    if (area.size() > 3) {
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
                    _lightBarList.push_back(LightBar((contour[0] + contour[1]) / 2 + ROIoffset, (contour[2] + contour[3]) / 2 + ROIoffset, 0));
                    return true;
                }
                else if (270 - angleRange < angle && angle < 270 + angleRange) {
                    _lightBarList.push_back(LightBar((contour[2] + contour[3]) / 2 + ROIoffset, (contour[0] + contour[1]) / 2 + ROIoffset, 0));
                    return true;
                }
            }
            else if (diff < 0) {  //旋转前，矩形横放
                float angle = fmod(box.angle + 360 + 90, 360);
                if (90 - angleRange < angle && angle < 90 + angleRange) {
                    _lightBarList.push_back(LightBar((contour[1] + contour[2]) / 2 + ROIoffset, (contour[0] + contour[3]) / 2 + ROIoffset, 0));
                    return true;
                }
                else if (270 - angleRange < angle && angle < 270 + angleRange) {
                    _lightBarList.push_back(LightBar((contour[0] + contour[3]) / 2 + ROIoffset, (contour[1] + contour[2]) / 2 + ROIoffset, 0));
                    return true;
                }
            }
        }
    }    
    return false;
}

inline void ArmorIdentifier_V2::_matchArmorPlates(const Img& imgGray, std::vector<ArmorPlate>& result) {
    /*result.clear();
    std::sort(_lightBarList.begin(), _lightBarList.end(),
        [&](LightBar& a, LightBar& b) {
            return a.top.x < b.top.x;
        });
    if (_lightBarList.size() >= 2)
        for (auto i = _lightBarList.begin() + 1; i != _lightBarList.end(); ++i) {
            result.push_back(ArmorPlate(*(i - 1), *i, 3));
        }*/
    result.clear();
    std::sort(_lightBarList.begin(), _lightBarList.end(),
        [&](LightBar& a, LightBar& b) {
        return a.top.x < b.top.x;
    }
    );
    int n = _lightBarList.size();
    for (int i = 0; i < n; ++i) {
        float Isize = P2PDis(_lightBarList[i].top, _lightBarList[i].bottom);
        cv::Point2f Icenter = (_lightBarList[i].top + _lightBarList[i].bottom) / 2;
        for (int j = i + 1; j < n; ++j) { // 一些筛选条件
            float Jsize = P2PDis(_lightBarList[j].top, _lightBarList[j].bottom);
            if (max(Isize, Jsize) / min(Isize, Jsize) > maxArmorLightRatio)		continue;
            if (fabs(_lightBarList[i].angle - _lightBarList[j].angle) > maxdAngle)	continue;
            if (malposition(_lightBarList[i], _lightBarList[j]) > maxMalposition)		continue;
            cv::Point2f Jcenter = (_lightBarList[j].top + _lightBarList[j].bottom) / 2;
            if (fabs(Icenter.y - Jcenter.y) * 2 / (Isize + Jsize) > maxLightDy)	continue;
            if (P2PDis(Icenter, Jcenter) * 2 / (Isize + Jsize) > bigArmorDis)	continue;

            // 数字识别部分（暂时放这，可能会挪到运动模型那去）
            ArmorPlate armor(_lightBarList[i], _lightBarList[j]);
            armor.id = _numberIdentifier.Identify(imgGray, armor);
            result.push_back(armor);
        }
    }
}

void ArmorIdentifier_V2::Identify(const Img& imgThre, const Img& imgGray, std::vector<ArmorPlate>& result) {
    auto areaList = _floodFindAreas(imgThre, 250, 120, true);
    _lightBarList.clear();
    for (auto i = areaList.begin(); i != areaList.end(); ++i)
        _solveToLightbar(*i);
    _matchArmorPlates(imgGray, result);

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

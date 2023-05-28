#pragma once
/*
Creation Date: 2022/11/23
Latest Update: 2022/11/23
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 绘制变量曲线图像
*/
#include <opencv2/opencv.hpp>
#include "Common/TimeStamp/TimeStampCounter.h"
#include "DebugSettings.h"
#include "Common/Color.h"

class ValueGragh :public cv::Mat {
    bool _initialized;
    double minX, maxX, minY, maxY;

    template<class ValType, size_t Size>
    friend class ValueGraghPrinter;
public:
    ValueGragh(const cv::Mat& img) : cv::Mat(img),
        _initialized(false), minX(.0), maxX(.0), minY(.0), maxY(.0) {}
    ValueGragh(int width, int height) : cv::Mat(height, width, CV_8UC3, cv::Scalar()),
        _initialized(false), minX(.0), maxX(.0), minY(.0), maxY(.0) {}
};

template<class ValType, size_t Size>
class ValueGraghPrinter {
private:
    struct ValueWithTimeStamp {
        TimeStamp tim;
        ValType val;

        ValueWithTimeStamp() :tim(0ULL), val() {}
        ValueWithTimeStamp(TimeStamp tp, ValType value) :
            tim(tp), val(value) {}
    };

    CircularQueue<ValueWithTimeStamp, Size + 10> _queue;
public:
    void pop() { _queue.pop(); }
    void push_back(const ValType& val) { 
        if (_queue.size() == Size) _queue.pop();
        _queue.push_back(ValueWithTimeStamp(TimeStampCounter::GetTimeStamp(), val));
    }
    void draw(ValueGragh& img, cv::Scalar color, bool followRatio = false);
};

template<class ValType, size_t Size>
inline void ValueGraghPrinter<ValType, Size>::draw(ValueGragh& img, cv::Scalar color, bool followRatio) {
    if (!img._initialized || !followRatio)
    { // 自确定XY度量值
        TimeStamp minTim = ULLONG_MAX, maxTim = 0;
        double      minVal = DBL_MAX,    maxVal = DBL_MIN;
        for (const auto& valWithTp : _queue) {
            if (minTim > valWithTp.tim) minTim = valWithTp.tim;
            if (maxTim < valWithTp.tim) maxTim = valWithTp.tim;
            if (minVal > valWithTp.val) minVal = valWithTp.val;
            if (maxVal < valWithTp.val) maxVal = valWithTp.val;
        }
        img.minX = minTim; img.maxX = maxTim;
        img.minY = minVal; img.maxY = maxVal;
        img._initialized = true;
    }
    // else 根据已确定的度量值绘制
    double Xratio = static_cast<double>(img.cols) / (img.maxX - img.minX + 1);
    double Yratio = static_cast<double>(img.rows) / (img.maxY - img.minY + 1);
    //*/// 绘制坐标轴刻度线
    if (!img._initialized && followRatio) {
        // 没画，嘿嘿(^V^)
    }
    //*/// 绘制变量曲线
    cv::Point2f lastPoint((_queue.first().tim - img.minX) * Xratio,
        (_queue.first().val - img.minY) * Yratio);
    for (const auto& valWithTp : _queue) {
        cv::Point2f point((valWithTp.tim - img.minX) * Xratio,
            (valWithTp.val - img.minY) * Yratio);
        cv::line(img, lastPoint, point, color);
        lastPoint = point;
    }
    //*///
}

// ######################## 生成(默认)调试用图像 #
#define MAKE_GRAGH(width, height) { ValueGragh __img(width, height);
#define MAKE_GRAGH_DEFAULT          MAKE_GRAGH(1000, 500)
// ########################## 显示调试图像 #
#define SHOW_GRAGH(imgName)          cv::imshow(#imgName, __img); cv::waitKey(1);}
// #################### 将变量曲线插入图像 #
#define DEBUG_GRAGH_IMG(x, img, color, maxSize) \
            { static ValueGraghPrinter<decltype(x), maxSize> __gragh; \
            __gragh.push_back(x); \
            __gragh.draw(img, color);}
// ######## 将变量曲线插入宏生成的调试图像 #
#define GRAGH_ADD_VAR_SIZED(x, color, maxSize) DEBUG_GRAGH_IMG(x, __img, color, maxSize)
#define GRAGH_ADD_VAR(x, color) DEBUG_GRAGH_IMG(x, __img, color, DEFAULT_MEM_NUM)
// ############ 显示带参数调节的单变量曲线 #
#define DEBUG_GRAGH_PARA(x, width, height, color, maxSize) \
            { static ValueGragh __img(width, height); \
            static ValueGraghPrinter<decltype(x), maxSize> __gragh; \
            __gragh.push_back(x); \
            __gragh.draw(__img, color, true); \
            cv::imshow(#x, __img); cv::waitKey(1); \
            }
// #################### 显示默认单变量曲线 #
#define DEBUG_GRAGH(x) DEBUG_GRAGH_PARA(x, 1000, 500, Scalar(0, 0, 255), DEFAULT_MEM_NUM)

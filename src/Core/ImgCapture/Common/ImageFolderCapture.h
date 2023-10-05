#pragma once
/*
Creation Date: 2023/06/10
Latest Update: 2022/06/10
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 包装OpenCV自带VideoCapture，使其符合ImgCapture的接口
- 仅包含视频读入
*/

#include <iostream>
#include <filesystem>

#include <opencv2/opencv.hpp>

#include "Util/TimeStamp/TimeStampCounter.h"
#include "Util/Debug/DebugCanvas.h"

class ImageFolderCapture {
public:
    explicit ImageFolderCapture(const char* path) {
        using std::filesystem::directory_iterator;

        int count = 0;
        for (auto& v : directory_iterator(path))
        {
            if (std::filesystem::is_regular_file(v.path()) && v.path().extension().string() == ".png") {
                _imgList.push_back(v.path().string());
            }
        }
    }

    std::tuple<cv::Mat, TimeStamp> Read() {
        int diff;

        if constexpr (debugCanvas.master) {
            auto& handler = debugCanvas.master.DebugFrameHandler;
            diff = handler.FrameAdjust + (handler.Paused ? 0 : 1);
            handler.FrameAdjust = 0;
        }
        else diff = 1;


        std::tuple<cv::Mat, TimeStamp> result;
        auto& [img, timestamp] = result;

        if (!_imgList.empty()) {
            _imgIndex = (_imgIndex + diff) % _imgList.size();
            img = cv::imread(_imgList[_imgIndex]);
        }

        timestamp = TimeStampCounter::GetTimeStamp();
        return result;
    }

private:
    std::vector<std::string> _imgList;
    int _imgIndex = 0;
};

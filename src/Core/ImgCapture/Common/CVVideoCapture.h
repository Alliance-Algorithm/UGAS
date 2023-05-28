#pragma once
/*
Creation Date: 2022/10/12
Latest Update: 2022/03/17
Developer(s): 21-THY 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 包装OpenCV自带VideoCapture，使其符合ImgCapture的接口
- 仅包含视频读入
*/

#include "Core/ImgCapture/ImgCaptureInterface.h"
#include "Util/Debug/Log.h"
#include "Util/Debug/DebugCanvas.h"

class CVVideoCapture : public ImgCaptureInterface, private cv::VideoCapture {
private:
    int _frameCount;

public:
    explicit CVVideoCapture(const std::string& fileName) {
        VideoCapture::open(fileName);
        if (!isOpened()) {
            VideoCapture::open(std::string("resources/") + fileName);
            if (!isOpened())
                throw_with_trace(std::runtime_error, "Fail to open.");
        }
        _frameCount = static_cast<int>(VideoCapture::get(cv::CAP_PROP_FRAME_COUNT));
        if (_frameCount == 0)
            throw_with_trace(std::runtime_error, "Read empty video!")
    }
    CVVideoCapture(const CVVideoCapture&) = delete;
    CVVideoCapture(CVVideoCapture&&) = delete;

    std::tuple<cv::Mat, TimeStamp> Read() override {
        if constexpr(debugCanvas.master) {
            auto& handler = debugCanvas.master.DebugFrameHandler;
            int frameAdjust = handler.FrameAdjust;
            handler.FrameAdjust = 0;
            if (handler.Paused)
                --frameAdjust;
            if (frameAdjust != 0) {
                int frameIndex = static_cast<int>(VideoCapture::get(cv::CAP_PROP_POS_FRAMES)) + frameAdjust + (100 * _frameCount);
                frameIndex %= _frameCount;
                VideoCapture::set(cv::CAP_PROP_POS_FRAMES, frameIndex);
            }
        }        

        std::tuple<cv::Mat, TimeStamp> result;
        auto& [img, timestamp] = result;
        VideoCapture::read(img);
        if (img.empty()) {
            VideoCapture::set(cv::CAP_PROP_POS_FRAMES, 0);
            VideoCapture::read(img);
            if (img.empty()) {
                throw_with_trace(std::runtime_error, "Read empty frame!")
            }
        }
        cv::rotate(img, img, cv::ROTATE_180);
        timestamp = TimeStampCounter::GetTimeStamp();
        return result;
    }
};

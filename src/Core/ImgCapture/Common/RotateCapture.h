#pragma once
/*
Creation Date: 2023/03/21
Latest Update: 2023/03/21
Developer(s): 22-QZH
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- �ṩ������ImgCapture��ķ�װ����ԭCapture��Ļ�����������תͼ����
- ʹ�÷���(��CVVideoCapture��Ϊ��):
- ResizeCapture<CVVideoCapture> capture(0.5, "Blue_4.mp4");                //
*/

#include <opencv2/opencv.hpp>

template<typename CaptureType>
class RotateCapture : public CaptureType {
public:
    cv::RotateFlags _rotateFlag;

    RotateCapture() = delete;

    template<typename... Types>
    RotateCapture(cv::RotateFlags rotateFlag, Types &&... args) : CaptureType(std::forward<Types>(args)...) {
        _rotateFlag = rotateFlag;
    }

    RotateCapture(const RotateCapture &) = delete;

    RotateCapture(RotateCapture &&) = delete;

    std::tuple<cv::Mat, TimeStamp> Read() override {
        auto tuple = CaptureType::Read();
//TODO�� fix constexpr related bug, temporally delete this
        if (debugCanvas.master) {
            if (debugCanvas.master.DebugFrameHandler.Paused)
                return tuple;
        }
        auto &[mat, timestamp] = tuple;
        cv::rotate(mat, mat, _rotateFlag);
        return tuple;
    }
};
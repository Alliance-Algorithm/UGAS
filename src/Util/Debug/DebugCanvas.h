#pragma once
/*
Creation Date: 2022/11/20
Latest Update: 2023/1/4
Developer(s): 21-THY 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 用于管理、显示调试图像
*/
#include <list>

#include <opencv2/opencv.hpp>

#include "config.h"
#include "MatForm/MatForm.hpp"
#include "MatForm/RectangleControl.hpp"
#include "MatForm/ButtonControl.hpp"

template<bool Enabled>
class ControlCanvas : private MatForm {
private:
    //RectangleControl _ROI;
    ButtonControl _btnForward10Frame, _btnForwardFrame, _btnPause, _btnNextFrame, _btnNext10Frame;

    int _lastFrameRows = 0, _lastFrameCols = 0;

public:
    constexpr operator bool () const {
        return true;
    }

    struct FrameHandler {
        bool Paused;
        int FrameAdjust;

        FrameHandler() {
            Paused = false;
            FrameAdjust = 0;
        }
    } DebugFrameHandler;

    ControlCanvas() = delete;

    template<typename... Types>
    ControlCanvas(Types&&... args) : MatForm(std::forward<Types>(args)...) {
        _btnForward10Frame.Text = "<<";
        _btnForward10Frame.OnMouseDown = [this](auto) {
            DebugFrameHandler.FrameAdjust -= 10;
        };
        AddControl(&_btnForward10Frame);

        _btnForwardFrame.Text = "<";
        _btnForwardFrame.OnMouseDown = [this](auto) {
            DebugFrameHandler.FrameAdjust--;
        };
        AddControl(&_btnForwardFrame);

        _btnPause.Text = "Pause";
        _btnPause.OnMouseDown = [this](auto) {
            if (DebugFrameHandler.Paused) {
                DebugFrameHandler.Paused = false;
                _btnPause.Text = "Pause";
            }
            else {
                DebugFrameHandler.Paused = true;
                _btnPause.Text = "Play";
            }
        };
        AddControl(&_btnPause);

        _btnNextFrame.Text = ">";
        _btnNextFrame.OnMouseDown = [this](auto) {
            DebugFrameHandler.FrameAdjust++;
        };
        AddControl(&_btnNextFrame);

        _btnNext10Frame.Text = ">>";
        _btnNext10Frame.OnMouseDown = [this](auto) {
            DebugFrameHandler.FrameAdjust += 10;
        };
        AddControl(&_btnNext10Frame);

        //_ROI.ForeColor = COLOR_YELLOW;
        //AddControl(&_ROI);
    }

    void LoadMat(const cv::Mat& img) {
        MatForm::LoadMat(img.clone());
        if (_lastFrameRows != rows || _lastFrameRows != cols) {
            //更新按钮位置
            _btnForward10Frame.BoundRect = cv::Rect(cols - 300, rows - 60, 60, 60);
            _btnForwardFrame.BoundRect = cv::Rect(cols - 240, rows - 60, 60, 60);
            _btnPause.BoundRect = cv::Rect(cols - 180, rows - 60, 60, 60);
            _btnNextFrame.BoundRect = cv::Rect(cols - 120, rows - 60, 60, 60);
            _btnNext10Frame.BoundRect = cv::Rect(cols - 60, rows - 60, 60, 60);
        }
        //_ROI.BoundRect = roiRect;
    }

    constexpr cv::Mat& GetMat() {
        return *this;
    }

    constexpr MatForm& GetMatForm() {
        return *this;
    }
};

template<>
class ControlCanvas<false> {
public:
    struct FrameHandler {
        bool Paused;
        int FrameAdjust;

        FrameHandler() {
            Paused = false;
            FrameAdjust = 0;
        }
    } DebugFrameHandler;

    template<typename... Types>
    ControlCanvas(Types&&... args) { }

    constexpr operator bool () const { return false; }

    void LoadMat(const cv::Mat& img);
    constexpr cv::Mat& GetMat();
};


template<bool Enabled>
class SimpleCanvas : private MatForm {
public:
    template<typename... Types>
    SimpleCanvas(Types&&... args) : MatForm(std::forward<Types>(args)...) { }

    constexpr operator bool () const { return true; }

    void LoadMat(const cv::Mat& img) {
        MatForm::LoadMat(img.clone());
    }

    constexpr cv::Mat& GetMat() {
        return *this;
    }
};

template<>
class SimpleCanvas<false>{
public:
    template<typename... Types>
    SimpleCanvas(Types&&... args) { }

    constexpr operator bool () const { return false; }

    void LoadMat(const cv::Mat& img);
    constexpr cv::Mat& GetMat();
};


template<typename Target, bool Enabled>
class ReferenceCanvas {
public:

    ReferenceCanvas() = delete;
    ReferenceCanvas(Target& canvas) { }

    constexpr operator bool() const { return false; }

    constexpr cv::Mat& GetMat();
};

template<>
class ReferenceCanvas<ControlCanvas<true>, true>{
private:
    cv::Mat& _target;

public:
    ReferenceCanvas() = delete;
    ReferenceCanvas(ControlCanvas<true>& canvas) : _target(canvas.GetMat()) { }

    constexpr operator bool() const { return true; }

    constexpr cv::Mat& GetMat() {
        return _target;
    }
};

template<>
class ReferenceCanvas<SimpleCanvas<true>, true> {
private:
    cv::Mat& _target;

public:
    ReferenceCanvas() = delete;
    ReferenceCanvas(SimpleCanvas<true>& canvas) : _target(canvas.GetMat()) { }

    constexpr operator bool() const { return true; }

    constexpr cv::Mat& GetMat() {
        return _target;
    }
};


template<bool Enabled>
class ReadonlyCanvas : private MatForm {
public:
    template<typename... Types>
    ReadonlyCanvas(Types... args) : MatForm(std::forward<Types>(args)...) { }

    constexpr operator bool() const { return true; }

    void LoadMat(const cv::Mat& img) {
        //Note: 只在类内部去除const约束，不要在类内对img做任何修改
        MatForm::LoadMat(const_cast<cv::Mat&>(img));
    }

    constexpr const cv::Mat& GetMat() {
        return *this;
    }
};

template<>
class ReadonlyCanvas<false> {
public:
    template<typename... Types>
    ReadonlyCanvas(Types... args) { }

    constexpr operator bool() const { return false; }

    void LoadMat(const cv::Mat& img);
    constexpr const cv::Mat& GetMat();
};


#define MAKE_CONTROL_CANVAS(enable, name, arg) ControlCanvas<ENABLE_DEBUG_CANVAS && enable> name = ControlCanvas<ENABLE_DEBUG_CANVAS && enable>(arg, &_matFormList);
#define MAKE_SIMPLE_CANVAS(enable, name, arg) SimpleCanvas<ENABLE_DEBUG_CANVAS && enable> name = SimpleCanvas<ENABLE_DEBUG_CANVAS && enable>(arg, &_matFormList);
#define MAKE_REFERENCE_CANVAS(enable, name, target) ReferenceCanvas<decltype(target), ENABLE_DEBUG_CANVAS && enable> name = target;
#define MAKE_READONLY_CANVAS(enable, name, arg) ReadonlyCanvas<ENABLE_DEBUG_CANVAS && enable> name = ReadonlyCanvas<ENABLE_DEBUG_CANVAS && enable>(arg, &_matFormList);

class DebugCanvas {
private:
    std::list<MatForm*> _matFormList;

public:

    MAKE_CONTROL_CANVAS   (1, master, "Master Image")          // 主调试图像
    MAKE_REFERENCE_CANVAS (1, lightbar, master)                // 灯条识别结果
    MAKE_REFERENCE_CANVAS (1, armor, master)                   // 装甲板识别结果
    MAKE_REFERENCE_CANVAS (1, armorNum, master)                // 装甲板数字识别结果
    MAKE_REFERENCE_CANVAS (1, track, master)                   // 跟踪点
    MAKE_REFERENCE_CANVAS (1, predict, master)                 // 预测点
    MAKE_REFERENCE_CANVAS (1, fps, master)                     // 帧率
    MAKE_READONLY_CANVAS  (0, pretreat, "Pretreat Image")      // 预处理后的图像

    void ShowAll() {
        for (const auto ptrForm : _matFormList) {
            ptrForm->Show();
        }
    }
};
/*
debugCanvas的标准使用格式:
if constexpr(debugCanvas.xxx) {...}
如果报编译错误，请检查你的代码有没有漏掉constexpr，或者忘写if
*/
extern DebugCanvas debugCanvas;
 
#undef MAKE_CONTROL_CANVAS
#undef MAKE_SIMPLE_CANVAS
#undef MAKE_REFERENCE_CANVAS
#undef MAKE_READONLY_CANVAS
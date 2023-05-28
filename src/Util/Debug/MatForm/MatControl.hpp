#pragma once
/*
Creation Date: 2023/1/4
Latest Update: 2023/1/4
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 控件基类，可添加进MatForm
*/
#include "MatForm.hpp"

class MatForm;

class MatControl {
protected:
    MatForm* _parent;

public:
    cv::Rect BoundRect;

    struct MouseEventArg {
        int x, y;
        MouseEventArg(int x, int y) {
            this->x = x;
            this->y = y;
        }
    };

    MatControl() {
        _parent = nullptr;
        BoundRect = cv::Rect(0, 0, 10, 10);
    }

    MatControl(const cv::Rect& rect) {
        _parent = nullptr;
        BoundRect = rect;
    }

    MatControl(const int x, const int y, const int w, const int h) {
        _parent = nullptr;
        BoundRect = cv::Rect(x, y, w, h);
    }

    virtual ~MatControl() { }

    inline void SetParent(MatForm* parent) {
        _parent = parent;
    }

    virtual void Draw() const = 0;

    std::function<void(MouseEventArg)> OnMouseDown;
    std::function<void(MouseEventArg)> OnMouseUp;
};
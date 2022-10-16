#pragma once
/*
Creation Date: 2022/10/16
Latest Update: 2022/10/16
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 提供统计帧数信息的方法
*/
#include "../../../Parameters/DebugSettings.h"
#include "../TimeStamp/TimeStampCounter.h"
#include "../UniversalStruct.h"

template <class valType, int Size>
class CircularQueue {
    size_t _head, _last;
    valType data[Size];
public:
    CircularQueue() :_head(0), _last(0) {}
    void push(valType v) {
        if (_last == Size) _last = 0;
        data[_last++] = v;
    }
    size_t size() {
        return (_last >= _head) ? _last - _head :
            Size + _last - _head;
    }
    bool empty() { return _head == _last; }
    void pop() { if (++_head == Size) _head = 0; }
    valType& front() { return data[_head]; }
    valType& back() { return data[_last - 1]; }
};

class FPSCounter {
private:
	CircularQueue<TimeStamp, MAX_FPS> _timeStamps;
public:
	void Count();
	int GetFPS();
	void PrintFPS(cv::Mat& img);
};

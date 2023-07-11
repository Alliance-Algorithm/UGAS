#pragma once
/*
Creation Date: 2022/10/16
Latest Update: 2023/05/08
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 提供统计帧数信息的方法
*/
#include "Util/TimeStamp/TimeStampCounter.h"
#include "Util/Parameter/Parameters.h"
#include "Util/Debug/Log.h"


class FPSCounter_V1 {
private:
    template <class valType, int Size>
    class CircularQueue {
    private:
        valType _data[Size];
        int _head, _tail;
    public:
        CircularQueue() :_head(0), _tail(0) {}
        class iterator;

        const valType& at(int index) const { return _data[index]; }
        void push_back(const valType& val) {
            _data[_tail++] = val;
            if (_tail == _head)
                throw_with_trace(std::runtime_error, "overflow in push_back()!");
            if (_tail == Size) _tail = 0;
        }
        void pop() { if (++_head == Size) _head = 0; }
        void clear() { _head = _tail = 0; }
        bool empty() const { return _tail == _head; }
        int size()   const { return _tail < _head ? (Size + _tail - _head) : (_tail - _head); }
        const valType& first() const { return _data[_head]; }
        const valType& last()  const { return _data[_tail ? _tail : (Size - 1)]; }
        inline iterator begin() { return iterator(*this, _head); }
        inline iterator end() { return iterator(*this, _tail); }
        inline int get_next(int index) const { return ++index == Size ? 0 : index; }

        class iterator {
            CircularQueue& _queue;
            int _index;
        public:
            iterator(CircularQueue& queue) :
                _queue(queue), _index(queue._head) {}
            iterator(CircularQueue& queue, int index) :
                _queue(queue), _index(index) {}

            iterator& operator ++() {
                _index = _queue.get_next(_index);
                return *this;
            }
            bool operator !=(iterator another) const
            {
                return _index != another._index ||
                    &_queue != &another._queue;
            }
            const valType& operator*() const { return _queue.at(_index); }
        };
    };

    static constexpr int MaxFps = 1000;
    CircularQueue<TimeStamp, MaxFps + 100> _timeStamps;

public:
    int Count();
    void PrintFPS(cv::Mat& img);
};

inline int FPSCounter_V1::Count() {
    if (!_timeStamps.empty()) {
        int _millisecDif = 1000. / MaxFps - (TimeStampCounter::GetTimeStamp() - _timeStamps.last());
        if (_millisecDif > 0) cv::waitKey(_millisecDif);
    }

    TimeStamp _presentTime = TimeStampCounter::GetTimeStamp();
    _timeStamps.push_back(_presentTime);
    while (!_timeStamps.empty() &&
        _presentTime - _timeStamps.first() > 1000)
        _timeStamps.pop();
    return _timeStamps.size();
}

inline void FPSCounter_V1::PrintFPS(cv::Mat& img) {
    cv::putText(img, "Fps : " + std::to_string(_timeStamps.size()),
        cv::Point(0, 25), 0, 1, COLOR_LIME);
}


class FPSCounter_V2 {
public:
    bool Count() {
        if (_count == 0) {
            _count = 1;
            _timingStart = std::chrono::steady_clock::now();
        }
        else {
            ++_count;
            if (std::chrono::steady_clock::now() - _timingStart >= std::chrono::seconds(1)) {
                _lastFPS = _count;
                _count = 0;
                return true;
            }
        }
        return false;
    }

    int GetFPS() {
        return _lastFPS;
    }

private:
    int _count = 0, _lastFPS;
    std::chrono::steady_clock::time_point _timingStart;
};
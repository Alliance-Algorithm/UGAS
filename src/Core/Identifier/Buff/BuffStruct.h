#pragma once
/*
Creation Date: 2023/5/22
Latest Update: 2023/5/22
Developer(s): 21-WZY 21-WTR
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- data structure for power rune
*/

#include <opencv2/opencv.hpp>

#include "Util/TimeStamp/TimeStampCounter.h"
#include "Util/Debug/Log.h"


enum class BuffMode { UnknownBuff = 0, SmallBuff, BigBuff };

// waring: asynchronous construction with evaluation

// 5 * 2D points
struct Buff5Point {
    std::vector<cv::Point2f> points;
    
    // 0:RightUP 1:LeftUP 2:LeftLOW 3:RightLOW 4:rCenter
    void Set(const cv::Point2f rightUP, const cv::Point2f leftUP,
             const cv::Point2f leftLOW, const cv::Point2f rightLOW,
             const cv::Point2f rCenter) {
        points.clear();
        points.push_back(rightUP);
        points.push_back(leftUP);
        points.push_back(leftLOW);
        points.push_back(rightLOW);
        points.push_back(rCenter);
    }
};

struct BuffAngle {
	TimeStamp timeStamp;
	float angle;

	BuffAngle() {}
	BuffAngle(TimeStamp ts, float a) : timeStamp(ts), angle(a) {}
};

struct BuffAngularSpeed {
	TimeStamp timeStamp;
	float speed;

	BuffAngularSpeed() {}
	BuffAngularSpeed(TimeStamp ts, float s) : timeStamp(ts), speed(s) {}
};

// 5 * 2D points identify data
struct Buff5PointIdentifyData {
	cv::Point2f rightUp;
	cv::Point2f leftUp;
	cv::Point2f leftLow;
	cv::Point2f rightLow;
	cv::Point2f rCenter;
	float radius;

	BuffAngle buffAngle;
	BuffAngularSpeed buffAngularSpeed;

	Buff5PointIdentifyData() {}
	Buff5PointIdentifyData(cv::Point2f rightUP, cv::Point2f leftUP,
						   cv::Point2f leftLOW, cv::Point2f rightLOW, 
						   cv::Point2f Center, float r,
						   BuffAngle ba, BuffAngularSpeed bas) :
		rightUp(rightUP), leftUp(leftUP), leftLow(leftLOW), rightLow(rightLOW),
		rCenter(Center), radius(r), buffAngle(ba), buffAngularSpeed(bas) {}
};

struct BuffFitData {
	TimeStamp FitTime;
	float A;
	float B;
	float C;
	float D;

	BuffFitData() {}
	BuffFitData(TimeStamp ft, float a, float b, float c, float d) :
		FitTime(ft), A(a), B(b), C(c), D(d) {}
};

template <class valType, int Size>
class CircularQueue {
private:
	valType _data[Size];
	int _head, _tail;
public:
	CircularQueue() :_head(0), _tail(0) {}
	class iterator;

	const valType& at(int index) const { return _data[index]; }
	const valType& read(int index)const {
		return _head + index < Size ? _data[_head + index] : _data[_head + index - Size];
	}
	void push_back(const valType& val) {
		_data[_tail++] = val;
		if (_tail == _head)
			throw_with_trace(std::runtime_error, "overflow in push_back()!");
		if (_tail == Size) _tail = 0;
	}
	void pop_back() { if (--_tail == -1) _tail = Size - 1; }
	void pop() { if (++_head == Size) _head = 0; }
	void clear() { _head = _tail = 0; }
	bool empty() const { return _tail == _head; }
	int size()   const { return _tail < _head ? (Size + _tail - _head) : (_tail - _head); }
	void disp() { std::cout << (_tail < _head ? (Size + _tail - _head) : (_tail - _head)) << "(" << _head << " - " << _tail << ")" << std::endl; }
	const valType& first() const { return _data[_head]; }
	const valType& last()  const { return _data[_tail ? _tail - 1 : (Size - 1)]; }
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

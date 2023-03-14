#pragma once
/*
Creation Date: 2022/10/11
Latest Update: 2022/10/13
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 声明并定义所有通用结构体
*/
#include <opencv2/opencv.hpp>
#include "DebugTools/Implementations/Easylogging++.h"
#include "DebugTools/Implementations/CustomException.h"
#include "DebugTools/Implementations/ExceptionHandling.h"
/* 
* 值得说的是，因为这个头文件被很多文件引用，
* 所以不要include不必要的东西，容易循环引用，
* 如果你遇到了类似“未定义的标识符”、“不允许使用不完整的类型”，
* 请检查头文件之间的引用关系，尽量避免出现循环引用的情况。
*/

enum Team { Red = 1, Blue = 2 };

typedef unsigned long long TimeStamp;

/*class MatWithTimeStamp :public cv::Mat {
public:
	TimeStamp timeStamp;

	MatWithTimeStamp() = delete;
	explicit MatWithTimeStamp(TimeStamp ts) : timeStamp(ts) {}
	explicit MatWithTimeStamp(const Mat& img, TimeStamp ts) :Mat(img), timeStamp(ts) {}

	void SetROI(const cv::Rect& ROI) { static_cast<Mat&>(*this) = this->operator()(ROI); }
};
typedef MatWithTimeStamp Img;*/

struct LightBar {
	cv::Point2f top, bottom;
	float angle;

	// 这个自动计算Angle的实现要和识别的地方表示方式同步
	// 看看什么时候搞个统一的标准还是怎么，先不实现这个构造
	LightBar(cv::Point2f _top, cv::Point2f _bottom);
	LightBar(cv::Point2f _top, cv::Point2f _bottom, float angle) :
		top(_top), bottom(_bottom), angle(angle) {}
};

class ArmorPlate {
public:
	std::vector<cv::Point2f> points;
	short id;

	ArmorPlate() :id(0) {} // 用来支持数组
	ArmorPlate(const LightBar& left, const LightBar& right, short _id = 0):
		id(_id) { Set(left, right, _id); }

	// U字型（和PNP参数表示顺序同步（老代码传承的顺序））
	void Set(const LightBar& left, const LightBar& right, short _id = 0) {
		points.push_back(left.top); points.push_back(left.bottom);
		points.push_back(right.bottom); points.push_back(right.top);
		if (_id) id = _id;
	}
	cv::Point2f center() const {
		if (points.size() != 4)
			throw_with_trace(std::runtime_error, "Invalid ArmorPlate object");
		return (points[0] + points[1] + points[2] + points[3]) / 4;
	}
	
	std::vector<cv::Point2f>& OffsetPoints(const cv::Point2f offset) const {
		//TODO 生存期过了，要改非复制传参
		std::vector<cv::Point2f> offsetPoints;
		for_each(points.begin(), points.end(), [&](const cv::Point2f& point) {
			offsetPoints.push_back(point + offset);
		});
		return offsetPoints;
	}
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
	inline iterator end()   { return iterator(*this, _tail); }
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

#pragma once

template <class T>
class FastStack {
private:
	T* _pointer;
	int _length, _maxLength;
public:
	int& Length = _length;

	FastStack() {
		_length = 0;
		_pointer = nullptr;
	}

	~FastStack() {
		if (_pointer != nullptr)
			delete[] _pointer;
	}

	inline void Reset() {
		_length = 0;
	}

	inline void Reset(int maxLength) {
		if (maxLength != _maxLength) {
			_maxLength = maxLength;
			if (_pointer != nullptr)
				delete[] _pointer;
			_pointer = new T[_maxLength];
		}
		Reset();
	}

	inline T* Push(T data) {
		_pointer[_length] = data;
		return &_pointer[_length++];
	}

	inline T& Pop() {
		return _pointer[--_length];
	}

	inline T& Top() const {
		return _pointer[_length - 1];
	}

	inline T& operator[](int index) {
		return _pointer[index];
	}

	inline const T& operator[](int index) const {
		return _pointer[index];
	}
};

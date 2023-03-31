#pragma once
#include <string.h>
#include <stdlib.h>

template <class T>
class FastArray2d {
private:
	T* _pointer;
	int _rows, _cols;
	size_t _memorySize;
public:
	FastArray2d() {
		_rows = _cols = 0;
		_pointer = nullptr;
	}

	~FastArray2d() {
		if (_pointer != nullptr)
			free(_pointer);
	}

	inline void Reset() {
		if (_pointer != nullptr)
			memset(_pointer, 0, _memorySize);
	}

	inline void Reset(int rows, int cols) {
		if (_rows != rows || cols != _cols) {
			_rows = rows;
			_cols = cols;
			if (_pointer != nullptr)
				free(_pointer);
			_memorySize = sizeof(T) * _rows * _cols;
			_pointer = (T*)malloc(_memorySize);
		}
		Reset();
	}

	inline T& At(int row, int col) {
		return _pointer[(row * _cols) + col];
	}
};

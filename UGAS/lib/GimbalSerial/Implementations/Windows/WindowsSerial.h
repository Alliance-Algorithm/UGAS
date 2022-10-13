#pragma once
/*
Creation Date: 2022/10/11
Latest Update: 2022/10/11
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 提供Windows串口通讯基本函数
*/
#ifdef _WIN32
#include <iostream>
#include <Windows.h>

namespace serial {
	class WindowsSerial {
	protected:
		bool _initialized;
		bool _synState;
		HANDLE _handle;
	public:
		WindowsSerial() :_initialized(false) {}
		WindowsSerial(const char* portName, int baudrate, BYTE parity, char databit, BYTE stopbit, bool synFlag)
			{ Open(portName, baudrate, parity, databit, stopbit, synFlag); }
		virtual ~WindowsSerial() { Close(); }

		bool Open(const char* portName, int baudrate, BYTE parity, char databit, BYTE stopbit, bool synFlag);
		void Close();
		bool IsOpen() { return _initialized; }
		bool send(const BYTE* data, int dataLenth);
		DWORD recv(BYTE* data, int dataMaxLenth);
	};
}
#endif
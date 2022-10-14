#ifdef _WIN32
#include "WindowsSerial.h"
using namespace serial;

bool WindowsSerial::Open(const char* portName, int baudrate, BYTE parity, char databit, BYTE stopbit, bool synFlag) {
#if VIRTUAL_GIBAL == 1
	return false;
#endif
	_handle = CreateFileA(portName, GENERIC_READ | GENERIC_WRITE, 0, NULL,
		OPEN_EXISTING, synFlag ? 0 : FILE_FLAG_OVERLAPPED, 0);
	if (_handle == INVALID_HANDLE_VALUE) {
		int errorCode = static_cast<int>(GetLastError());
		printf("[ERROR]<WindowsSerial> INVALID_HANDLE_VALUE in Open! Code: %d\n", errorCode);
		switch (errorCode) {
		case 2: puts("[ERROR]<WindowsSerial> Check the correctness of serial port name!"); break;
		case 5: puts("[ERROR]<WindowsSerial> Check whether the serial port has been occupied!"); break;
		default: throw "[ERROR]<WindowsSerial> INVALID_HANDLE_VALUE in Open!";
		}
		exit(errorCode);
	}
	if (!SetupComm(_handle, 1024, 1024)) {
		printf("[ERROR]<WindowsSerial> ERROR in SetupComm! Code: %d\n", GetLastError());
		throw "[ERROR]<WindowsSerial> ERROR in SetupComm!";
	}
	_synState = synFlag;

	DCB dcb;
	GetCommState(_handle, &dcb);
	dcb.BaudRate = baudrate;
	dcb.Parity = parity;
	dcb.ByteSize = databit;
	dcb.StopBits = stopbit;
	dcb.fRtsControl = RTS_CONTROL_ENABLE;
	if (!SetCommState(_handle, &dcb))
		throw "[ERROR]<WindowsSerial> ERROR in SetCommState!";

	COMMTIMEOUTS timeOuts;
	timeOuts.ReadIntervalTimeout = 1000;
	timeOuts.ReadTotalTimeoutMultiplier = 500;
	timeOuts.ReadTotalTimeoutConstant = 5000;
	timeOuts.WriteTotalTimeoutMultiplier = 500;
	timeOuts.WriteTotalTimeoutConstant = 2000;
	SetCommTimeouts(_handle, &timeOuts);

	PurgeComm(_handle, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);
	_initialized = true;
	return _initialized;
}

void WindowsSerial::Close() {
	if (_initialized) {
		CloseHandle(_handle);
		_initialized = false;
	}
}

bool WindowsSerial::send(const BYTE* data, int dataLenth) {
	if (!_initialized) return false;
	if (_synState) {
		DWORD sended;
		if (!WriteFile(_handle, data, dataLenth, &sended, NULL))
			return false;
		//printf("sended %d bits\n", dataLenth);
		return dataLenth == sended;
	}
	else {
		throw "[ERROR]<WindowsSerial> Undefined synState";
		puts("[ERROR]<WindowsSerial> Undefined synState");
		//暂时不打算异步
	}
	return false;
}

DWORD WindowsSerial::recv(BYTE* data, int dataMaxLenth) {
	if (!_initialized) return 0;
	if (_synState) {
		DWORD received;
		if (!ReadFile(_handle, data, dataMaxLenth, &received, NULL))
			return 0;
		return received;
	}
	else {
		throw "[ERROR]<WindowsSerial> Undefined synState";
		puts("[ERROR]<WindowsSerial> Undefined synState");
		//暂时不打算异步
	}
	return 0;
}
#endif
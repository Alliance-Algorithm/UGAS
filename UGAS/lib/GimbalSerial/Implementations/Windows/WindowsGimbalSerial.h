#pragma once
/*
Creation Date: 2022/10/11
Latest Update: 2022/10/11
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 封装Windows环境下云台通信有关功能
*/
#ifdef _WIN32
//#include <thread>
#include "../../GimbalSerial.h"
#include "WindowsSerial.h"

namespace serial {
	class WindowsGimbalSerial :public GimbalSerial, private WindowsSerial {
	protected:
		static void VerifyRecvData(RecvPkg& recvPkg);

		// 禁止复制
		WindowsGimbalSerial(WindowsGimbalSerial&) = default;
		WindowsGimbalSerial& operator=(WindowsGimbalSerial&) = default;
	public:
		WindowsGimbalSerial() :WindowsSerial() {}
		WindowsGimbalSerial(const char* portName) :
			WindowsSerial(portName, CBR_115200, NOPARITY, 8, ONESTOPBIT, true) {}
		WindowsGimbalSerial(WindowsSerial& windowsSerialPort) :
			WindowsSerial(windowsSerialPort) {}
		~WindowsGimbalSerial() { Close(); }

		void Open(const char* portName)
			{ WindowsSerial::Open(portName, CBR_115200, NOPARITY, 8, ONESTOPBIT, true); }
		bool IsOpen() { return WindowsSerial::IsOpen(); }
		void Close() { WindowsSerial::Close(); }

		bool  Send();
		const RecvPkg& RecvGimbalData();
	};
}
#endif // _WIN32
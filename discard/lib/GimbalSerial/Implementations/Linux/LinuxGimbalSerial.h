#pragma once
/*
Creation Date: 2022/10/24
Latest Update: 2022/10/24
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- GimbalSerial在Linux上的实例化，提供对Liunx串口的封装
- 参数定义见GimbalSerial.h
*/

#ifndef _WIN32

#include "../../GimbalSerial.h"
#include "../../Packages.h"
#include "LinuxSerial.h"

namespace serial {

	class LinuxGimbalSerial : public GimbalSerial, private LinuxSerial {
	private:
		const SerialOptions GimbalOptions;
		bool _isOpen = false;
	public:
		LinuxGimbalSerial() : GimbalOptions{ B115200, DataBits8, StopBits1, ParityNone } {}

		void Open(const char* portName);
		bool IsOpen();
		void Close();
		static void VerifyRecvData(RecvPkg& recvPkg) ;
		bool Send();
		const RecvPkg& RecvGimbalData();
	};
}

#endif
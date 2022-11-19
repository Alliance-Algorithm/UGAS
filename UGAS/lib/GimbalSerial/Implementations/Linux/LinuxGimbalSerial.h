#pragma once
/*
Creation Date: 2022/10/24
Latest Update: 2022/10/24
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- GimbalSerial��Linux�ϵ�ʵ�������ṩ��Liunx���ڵķ�װ
- ���������GimbalSerial.h
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
		LinuxGimbalSerial() : GimbalOptions{ 115200ul, DataBits8, StopBits1, ParityNone } {}

		void Open(const char* portName);
		bool IsOpen();
		void Close();

		bool Send();
		const RecvPkg& RecvGimbalData();
	};
}

#endif
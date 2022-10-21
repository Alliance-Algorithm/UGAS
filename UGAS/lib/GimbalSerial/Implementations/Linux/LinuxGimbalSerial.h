#pragma once
/*
Creation Date: 2022/10/21
Latest Update: 2022/10/21
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 没实现来充数的
*/
#ifndef _WIN32
#include "../../GimbalSerial.h"

namespace serial {
	class LinuxGimbalSerial : public GimbalSerial {
		void Open(const char* portName) {}
		bool IsOpen() { return 0; }
		void Close() {}

		bool  Send() { return 0; }
		const RecvPkg& RecvGimbalData() {
			return static_cast<RecvPkg&>(*this);
		}
	};
}
#endif
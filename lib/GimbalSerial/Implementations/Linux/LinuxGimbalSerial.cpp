#ifndef _WIN32

#include "LinuxGimbalSerial.h"
#include <DebugSettings.h>
#include <Parameters.h>
#include <Common/DebugTools/DebugHeader.h>
#include <errno.h>

namespace serial {
	void LinuxGimbalSerial::VerifyRecvData(RecvPkg& recvPkg) {
		if (IN_STATE(recvPkg.flag, STATE_SHUTDOWN))
		system("shutdown -s -t 0");
	//recvPkg.pitchA /= 10;
	//recvPkg.team = TEAM_BLUE;
	//recvPkg.speed = 26000;
	//recvPkg.flag |= STATE_LARGE;
}

	void LinuxGimbalSerial::Open(const char* portName) {
#if VIRTUAL_GIBAL == 0
		LinuxSerial::Open(portName, GimbalOptions);
#endif
		_isOpen = true;
	}
	bool LinuxGimbalSerial::IsOpen() {
		return _isOpen;
	}
	void LinuxGimbalSerial::Close() {
#if VIRTUAL_GIBAL == 0
		LinuxSerial::Close();
#endif
	}

	bool LinuxGimbalSerial::Send() {
#if VIRTUAL_GIBAL == 0
		if (!_isOpen) throw_with_trace(std::runtime_error, "Serial port is unavailable");
		LinuxSerial::Send((unsigned char*)static_cast<SendPkg*>(this), _SendPkgSize);
#endif
		return true;
	}

	const RecvPkg& LinuxGimbalSerial::RecvGimbalData() {
#if VIRTUAL_GIBAL == 0
		RecvPkg tmp;
		int len=11;
		int status=LinuxSerial::Read((unsigned char*) &tmp, _RecvPkgSize);//返回-1说明接受有问题
		//std::cout<<"收到的数据位数："<<status<<std::endl;

		/*测试小程序
		int status=read(_tty_fd,&head,1);
		std::cout<<status<<std::endl;
		std::cout<<"head0"<<int(head)<<std::endl;
		*/
		//tmp.Debug();
		if (tmp.head != '\xff') { // 重新对齐
			while (tmp.head != '\xff'){
				LinuxSerial::Read((BYTE*)&tmp, 1);
				// std::cout<<int(tmp.head)<<std::endl;
			}
			LinuxSerial::Read(((BYTE*)&tmp) + 1, _RecvPkgSize - 1);
		}
		//std::cout<<"end receive"<<std::endl;
		if (tmp.CheckCRC8()) {
			VerifyRecvData(tmp);
			static_cast<RecvPkg&>(*this) = tmp;
		}
#endif
		return static_cast<RecvPkg&>(*this);
	}
}

#endif
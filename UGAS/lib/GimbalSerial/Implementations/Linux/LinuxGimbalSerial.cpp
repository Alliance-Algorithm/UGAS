#if !defined(_WIN32)

#include "LinuxGimbalSerial.h"

namespace serial {

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
		LinuxSerial::Recv((unsigned char*)&tmp, _RecvPkgSize);

		if (tmp.head != '\xFF') { // ÖØÐÂ¶ÔÆë
			while (tmp.head != '\xFF')
				LinuxSerial::Recv((BYTE*)&tmp, 1);
			LinuxSerial::Recv(((BYTE*)&tmp) + 1, _RecvPkgSize - 1);
		}

		if (tmp.CheckCRC8()) {
			VerifyRecvData(tmp);
			static_cast<RecvPkg&>(*this) = tmp;
		}
#endif
		return static_cast<RecvPkg&>(*this);
	}
}

#endif
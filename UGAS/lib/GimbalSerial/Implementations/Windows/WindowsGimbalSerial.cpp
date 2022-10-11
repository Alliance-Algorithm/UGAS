#ifdef _WIN32
#include "WindowsGimbalSerial.h"
using namespace serial;

void WindowsGimbalSerial::VerifyRecvData(RecvPkg& recvPkg) {
	if (IN_STATE(recvPkg.flag, STATE_SHUTDOWN))
		system("shutdown -s -t 0");
	//recvPkg.pitchA /= 10;
	//recvPkg.team = TEAM_BLUE;
	//recvPkg.speed = 26000;
	//recvPkg.flag |= STATE_LARGE;
}

bool serial::WindowsGimbalSerial::Send() {
	if (!_initialized) return false;
	send((BYTE*)static_cast<SendPkg*>(this), _SendPkgSize);
	return true;
}

const RecvPkg& WindowsGimbalSerial::RecvGimbalData() {
	RecvPkg tmp;
	recv((BYTE*)&tmp, _RecvPkgSize);

	if (tmp.head != '\xFF') { // ÖØÐÂ¶ÔÆë
		while (tmp.head != '\xFF')
			recv((BYTE*)&tmp, 1);
		recv(((BYTE*)&tmp) + 1, _RecvPkgSize - 1);
	}

	if (tmp.CheckCRC8()) {
		VerifyRecvData(tmp);
		static_cast<RecvPkg&>(*this) = tmp;
	}
	return static_cast<RecvPkg&>(*this);
}
#endif // _WIN32

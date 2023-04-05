#pragma once

#include <chrono>

#include "Control/Serial/Serial.h"
#include "Control/Serial/CRC/CRC.h"
#include "Control/Gimbal/Gimbal.h"

class SerialCBoard {
public:

#pragma pack(push, 1)
	struct PackageSend {
		uint8_t 	head;					// 0xFF
		float	yaw, pitch;				// Angle Deviation
		uint8_t 	crc8;					// CRC8
		PackageSend(GimbalAttitude attitude) {
			head = 0xff; yaw = attitude.yaw; pitch = attitude.pitch;
			crc8 = Get_CRC8_Check_Sum((unsigned char*)this, sizeof(decltype(this)) - 1, 0xff);
		}
	};
#pragma pack(pop)

	SerialCBoard(const char* portName) : _serial(portName, UGAS_115200, NOPARITY, 8, ONESTOPBIT, true) { }

	void Send(GimbalAttitude attitude) {
		auto pkg = PackageSend(attitude);
		_serial.Send(&pkg, sizeof(PackageSend));
	}

private:
	Serial _serial;
};
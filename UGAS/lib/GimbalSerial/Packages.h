#pragma once
/*
Creation Date: 2022/10/11
Latest Update: 2022/10/11
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 定义串口数据包具体内容
- 声明数据包相关参数
*/
#include <iostream>
#include "CRC/CRC.h"
#include "Common/DebugTools/DebugHeader.h"
#include "Common/UniversalStruct.h"

// RecvPkg::flag States Define
#define STATE_NORMAL		0x01
#define STATE_BUFF			0x02
#define STATE_GUARD			0x04
#define STATE_SHUTDOWN		0x80

namespace serial {
	typedef unsigned char BYTE;
	typedef unsigned long DWORD;

#define IN_STATE(flag, stateDefine) \
	static_cast<bool>((flag)&(stateDefine))

#pragma pack(push, 1)
	struct SendPkg {
		char	head;					// 0xFF
		double	yaw, pitch;				// Angle Deviation
		char	crc8;					// CRC8

		void SetAngle(double _yaw, double _pitch) {
			head = '\xFF'; yaw = _yaw; pitch = _pitch;
			Append_CRC8_Check_Sum((BYTE*)this, sizeof(SendPkg));
		}
		void Debug(std::ostream& os = std::cout) const {
			os << "Yaw:" << yaw << " Pitch:" << pitch << std::endl;
		}
	};

	struct RecvPkg {
		char	head;					// 0xff
		char	team;					// team id
		float	yawA, pitchA, rollA;	// Angle
		float	yawW, pitchW, rollW;	// Angular Velocity
		float	speed;					// Bullet Speed
		char	flag;					// State flag
		char	crc8;					// CRC8

		bool CheckCRC8() const {
			return Verify_CRC8_Check_Sum((BYTE*)this, sizeof(RecvPkg));
		}
		void Debug(std::ostream& os = std::cout) const {
			if (CheckCRC8()) {
				os << "Team:" << ((team == Red) ? "Red" : "Blue") << std::endl;
				os << "yawA:" << yawA << "pitchA:" << pitchA << "rollA:" << rollA << std::endl;
				os << "yawW:" << yawW << "pitchW:" << pitchW << "rollW:" << rollW << std::endl;
				os << "speed:" << speed << "flag:" << static_cast<int>(flag) << std::endl;
			}
			else LOG(INFO) << "Invalid CRC8 Check Sum!";
		}
	};
#pragma pack(pop)

	static const char _SendPkgSize = sizeof(SendPkg);
	static const char _RecvPkgSize = sizeof(RecvPkg);
}

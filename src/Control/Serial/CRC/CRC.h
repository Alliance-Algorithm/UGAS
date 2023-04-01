#pragma once
/*
Creation Date: 2022/10/11
Latest Update: 2022/10/11
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 提供CRC8，CRC16校验码计算方法
*/
#include <cstdlib>
#include <cstdint>

extern "C" {
	// CRC8
	unsigned char Get_CRC8_Check_Sum(unsigned char* pchMessage, unsigned int dwLength, unsigned char ucCRC8);
	unsigned int Verify_CRC8_Check_Sum(unsigned char* pchMessage, unsigned int dwLength);
	void Append_CRC8_Check_Sum(unsigned char* pchMessage, unsigned int dwLength);

	// CRC16
	uint16_t Get_CRC16_Check_Sum(uint8_t* pchMessage, uint32_t dwLength, uint16_t wCRC);
	uint32_t Verify_CRC16_Check_Sum(uint8_t* pchMessage, uint32_t dwLength);
	void Append_CRC16_Check_Sum(uint8_t* pchMessage, uint32_t dwLength);
}

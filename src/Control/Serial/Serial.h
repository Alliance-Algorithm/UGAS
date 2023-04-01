#pragma once

#include "Control/Serial/Windows/WindowsSerial.h"

class Serial {
public:
	Serial(const char* portName) : _serial(portName, CBR_115200, NOPARITY, 8, ONESTOPBIT, true) {

	}

private:
	WindowsSerial _serial;
};
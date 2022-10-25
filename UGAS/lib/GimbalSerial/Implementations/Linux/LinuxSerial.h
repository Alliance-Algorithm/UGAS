#pragma once
/*
Creation Date: 2022/10/21
Latest Update: 2022/10/24
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- �ṩLinux����ͨѶ��������
*/
#ifndef _WIN32

#include <dirent.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include "../../GimbalSerial.h"
#include "../../../../Parameters/DebugSettings.h"
#include "../../../Common/DebugTools/DebugHeader.h"

namespace serial {
	enum DataBits : unsigned char {
		DataBits4 = 4,
		DataBits5 = 5,
		DataBits6 = 6,
		DataBits7 = 7,
		DataBits8 = 8,
	};

	enum StopBits : unsigned char {
		StopBits1 = 0,
		StopBits2 = 2
	};

	enum Parity : unsigned char {
		ParityNone = 0,
		ParityOdd = 1,
		ParityEven = 2,
		PariteMark = 3,
		ParitySpace = 4
	};

	struct SerialOptions {
		unsigned long baudRate;
		DataBits dataBits;
		StopBits stopBits;
		Parity parity;
	};

    class LinuxSerial {
    private:
        const bool _XON = false, _XOFF = false, _XANY = false;
        const int _VMIN = 0, _VTIME = 50;
        SerialOptions _open_options;
        int _tty_fd;
        bool _isOpen;
    protected:
        void termiosOptions(termios& tios, const SerialOptions& options);
    public:
        LinuxSerial() {}
        virtual ~LinuxSerial() { Close(); }

        void Open(const char* path, const SerialOptions& options);
        bool Send(const unsigned char* data, int length);
        int Read(unsigned char* data, int length);
        bool IsOpen();
        void Close();

        static int BaudRateMake(unsigned long baudrate);
        //static std::vector<std::string > list();
    };
}

#endif
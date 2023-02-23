#pragma once
/*
Creation Date: 2022/10/21
Latest Update: 2023/2/21
Developer(s): 22-Qzh 22-Iruma
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

namespace serial {
	enum DataBits : unsigned char {
		DataBits5 = CS5,
		DataBits6 = CS6,
		DataBits7 = CS7,
		DataBits8 = CS8,
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
		int baudRate;
		DataBits dataBits;
		StopBits stopBits;
		Parity parity;
	};

    class LinuxSerial {
    private:
        const bool _XON = false, _XOFF = false, _XANY = false;
        const int _VMIN = 0, _VTIME = 50;
        SerialOptions _open_options;
        bool _isOpen;
		
    protected:
        void termiosOptions(termios& tios, const SerialOptions& options);
    public:
	    int _tty_fd;
        LinuxSerial() {}
        virtual ~LinuxSerial() { Close(); }

        void Open(const char* path, const SerialOptions& options);
        bool Send(const unsigned char* data, int length);
        int Read(unsigned char *data, int length);
        bool IsOpen();
        void Close();


        //static std::vector<std::string > list();
    };
}

#endif
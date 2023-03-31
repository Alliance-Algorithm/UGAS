#ifndef _WIN32

#include "LinuxSerial.h"
#include <DebugSettings.h>
#include <Common/DebugTools/DebugHeader.h>

using namespace std;
using namespace serial;


void LinuxSerial::Open(const char* path, const SerialOptions& options) {
    _tty_fd = ::open(path, O_RDWR | O_NOCTTY );
    cout<<_tty_fd<<endl;
    if (_tty_fd < 0) {
        throw_with_trace(std::runtime_error, "Failed to open serial port")
    }

    struct termios tios;
    termiosOptions(tios, options);
    tcsetattr(_tty_fd, TCSANOW, &tios);
    tcflush(_tty_fd, TCIOFLUSH);

    _isOpen = true;
}

void LinuxSerial::termiosOptions(termios& tios, const SerialOptions& options) {
    if (_tty_fd < 0)
        throw_with_trace(std::runtime_error, "Serial port unavailable")

    cfmakeraw(&tios);
    tios.c_cflag |=(CLOCAL | CREAD);

    cfsetispeed(&tios, options.baudRate);
    cfsetospeed(&tios, options.baudRate);

    tios.c_iflag |= (_XON ? IXON : 0)
        | (_XOFF ? IXOFF : 0)
        | (_XANY ? IXANY : 0);

    // data bits
    tios.c_cflag &= ~ CSIZE;
    tios.c_cflag |= options.dataBits;

    // stop bits
    if (options.stopBits == StopBits2) {
        tios.c_cflag |= CSTOPB;
    }
    else {
        tios.c_cflag &= ~CSTOPB;
    }

    // parity
    if (options.parity == ParityNone) {
        tios.c_cflag &= ~PARENB;
    }
    else {
        tios.c_cflag |= PARENB;

        if (options.parity == PariteMark) {
            tios.c_cflag |= PARMRK;
        }
        else {
            tios.c_cflag &= ~PARMRK;
        }

        if (options.parity == ParityOdd) {
            tios.c_cflag |= PARODD;
        }
        else {
            tios.c_cflag &= ~PARODD;
        }
    }


}

bool LinuxSerial::IsOpen() {
    return _isOpen;
}

bool LinuxSerial::Send(const unsigned char* data, int length) {
    if (!_isOpen)
        throw_with_trace(std::runtime_error, "Serial port unavailable")
        return ::write(_tty_fd, (const void*)data, length) > -1;
}

int LinuxSerial::Read(unsigned char *data, int length) {
    if (!_isOpen)
        throw_with_trace(std::runtime_error, "Serial port unavailable")
    return ::read(_tty_fd, (void*)data, length);
}

void LinuxSerial::Close() {
    if (_isOpen) {
        ::close(_tty_fd);
        _isOpen = false;
    }
}

/*std::vector<std::string> LinuxSerial::list() {
    DIR* dir;
    struct dirent* ent;
    dir = opendir("/dev");
    std::vector<std::string> ttyList;

    while (ent = readdir(dir), ent != nullptr) {
        if ("tty" == std::string(ent->d_name).substr(0, 3)) {
            ttyList.emplace_back(ent->d_name);
        }
    }

    return ttyList;
}*/

#endif

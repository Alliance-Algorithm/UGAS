#ifndef _WIN32

#include "LinuxSerial.h"
using namespace serial;


void LinuxSerial::Open(const char* path, const SerialOptions& options) {
    _tty_fd = ::open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
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
    if (!_isOpen)
        throw_with_trace(std::runtime_error, "Serial port unavailable")

    tcgetattr(_tty_fd, &tios);

    cfmakeraw(&tios);
    tios.c_cflag &= ~(CSIZE | CRTSCTS);
    tios.c_iflag &= ~(IXON | IXOFF | IXANY | IGNPAR);
    tios.c_lflag &= ~(ECHOK | ECHOCTL | ECHOKE);
    tios.c_oflag &= ~(OPOST | ONLCR);

    cfsetispeed(&tios, options.baudRate);
    cfsetospeed(&tios, options.baudRate);

    tios.c_iflag |= (_XON ? IXON : 0)
        | (_XOFF ? IXOFF : 0)
        | (_XANY ? IXANY : 0);

    // data bits

    int databits[] = { CS5, CS6, CS7, CS8 };
    tios.c_cflag &= ~0x30;
    tios.c_cflag |= databits[options.dataBits];

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

    tios.c_cc[VMIN] = _VMIN;
    tios.c_cc[VTIME] = _VTIME;
}

bool LinuxSerial::IsOpen() {
    return _isOpen;
}

bool LinuxSerial::Send(const unsigned char* data, int length) {
    if (!_isOpen)
        throw_with_trace(std::runtime_error, "Serial port unavailable")
        return ::write(_tty_fd, (const void*)data, length) > -1;
}

int LinuxSerial::Read(unsigned char* data, int length) {
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

int LinuxSerial::BaudRateMake(unsigned long baudrate) {
    switch (baudrate) {
    case 50:
        return 0000001;
    case 75:
        return 0000002;
    case 110:
        return 0000003;
    case 134:
        return 0000004;
    case 150:
        return 0000005;
    case 200:
        return 0000006;
    case 300:
        return 0000007;
    case 600:
        return 0000010;
    case 1200:
        return 0000011;
    case 1800:
        return 0000012;
    case 2400:
        return 0000013;
    case 4800:
        return 0000014;
    case 9600:
        return 0000015;
    case 19200:
        return 0000016;
    case 38400:
        return 0000017;
    case 57600:
        return 0010001;
    case 115200:
        return 0010002;
    case 230400:
        return 0010003;
    case 460800:
        return 0010004;
    case 500000:
        return 0010005;
    case 576000:
        return 0010006;
    case 921600:
        return 0010007;
    case 1000000:
        return 0010010;
    case 1152000:
        return 0010011;
    case 1500000:
        return 0010012;
    case 2000000:
        return 0010013;
    case 2500000:
        return 0010014;
    case 3000000:
        return 0010015;
    case 3500000:
        return 0010016;
    case 4000000:
        return 0010017;
    default:
        throw_with_trace(std::invalid_argument, "Invaild baudrate!");
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

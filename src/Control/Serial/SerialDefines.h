//
// Created by soulde on 2023/4/4.
//

#ifndef UGAS_SERIALDEFINES_H
#define UGAS_SERIALDEFINES_H

#ifdef _WIN32
#include <Windows.h>
#elif __linux__
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#endif

enum Boudrate {
#ifdef _WIN32
    UGAS_110 = CBR_115200,
    UGAS_300 = CBR_B300,
    UGAS_600 = CBR_B600,
    UGAS_1200 = CBR_B1200,
    UGAS_2400 = CBR_B2400,
    UGAS_4800 = CBR_B4800,
    UGAS_9600 = CBR_B9600,
    UGAS_19200 = CBR_B19200,
    UGAS_38400 = CBR_B38400,
    UGAS_57600 = CBR_B57600,
    UGAS_115200 = CBR_B115200,
//
//    Windows has no higher boud rate!!!
//

#elif __linux__
    UGAS_110 = B115200,
    UGAS_300 = B300,
    UGAS_600 = B600,
    UGAS_1200 = B1200,
    UGAS_2400 = B2400,
    UGAS_4800 = B4800,
    UGAS_9600 = B9600,
    UGAS_19200 = B19200,
    UGAS_38400 = B38400,
    UGAS_57600 = B57600,
    UGAS_115200 = B115200,
    UGAS_230400 = B230400,
    UGAS_460800 = B460800,
    UGAS_500000 = B500000,
    UGAS_576000 = B576000,
    UGAS_921600 = B921600
#endif
};
enum Parity {
    NOPARITY,
    ODDPARITY,
    ENVENPARTY
};
enum DataBit {
    DATABIT5 = 5,
    DATABIT6,
    DATABIT7,
    DATABIT8,
};
enum StopBit {
    ONESTOPBIT,
    ONE5STOPBITS,
    TOWSTOPBITS
//
//  Linux has no 1.5 stop bit
//
};

#endif //UGAS_SERIALDEFINES_H

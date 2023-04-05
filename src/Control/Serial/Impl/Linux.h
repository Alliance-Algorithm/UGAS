//
// Created by soulde on 2023/4/5.
//
#pragma once

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

class Serial {

protected:
    bool _synState;
    int _handle;

public:
    Serial(const char *portName, uint32_t baudrate, uint8_t parity, uint8_t databit, uint8_t stopbit, bool synFlag) {

        _handle = open(portName, O_RDWR | O_NOCTTY);
        if (!_handle) {
            std::cout << "Serial " << portName << "start success" << std::endl;
        }

        fcntl(_handle, F_SETFL, 0);

        struct termios options{};
        cfmakeraw(&options);
        options.c_cflag |= (CLOCAL | CREAD);
        switch (parity) {
            // 无校验
            case 0:
                options.c_cflag &= ~PARENB;//PARENB：产生奇偶位，执行奇偶校验
                options.c_cflag &= ~INPCK;//INPCK：使奇偶校验起作用
                break;
                // 设置奇校验
            case 1:
                options.c_cflag |= PARENB;//PARENB：产生奇偶位，执行奇偶校验
                options.c_cflag |= PARODD;//PARODD：若设置则为奇校验,否则为偶校验
                options.c_cflag |= INPCK;//INPCK：使奇偶校验起作用
                options.c_cflag |= ISTRIP;//ISTRIP：若设置则有效输入数字被剥离7个字节，否则保留全部8位
                break;
                // 设置偶校验
            case 2:
                options.c_cflag |= PARENB;//PARENB：产生奇偶位，执行奇偶校验
                options.c_cflag &= ~PARODD;//PARODD：若设置则为奇校验,否则为偶校验
                options.c_cflag |= INPCK;//INPCK：使奇偶校验起作用
                options.c_cflag |= ISTRIP;//ISTRIP：若设置则有效输入数字被剥离7个字节，否则保留全部8位
                break;
            default: throw_with_trace(std::runtime_error, "unknown parity.");
        }
        switch (databit) {
            case 5:
                options.c_cflag &= ~CSIZE;//屏蔽其它标志位
                options.c_cflag |= CS5;
                break;
            case 6:
                options.c_cflag &= ~CSIZE;//屏蔽其它标志位
                options.c_cflag |= CS6;
                break;
            case 7:
                options.c_cflag &= ~CSIZE;//屏蔽其它标志位
                options.c_cflag |= CS7;
                break;
            case 8:
                options.c_cflag &= ~CSIZE;//屏蔽其它标志位
                options.c_cflag |= CS8;
                break;
            default: throw_with_trace(std::runtime_error, "unkown databit.");
        }

        switch (stopbit) {
            case 0:
                options.c_cflag &= ~CSTOPB;//CSTOPB：使用1位停止位
                break;
            case 2:
                options.c_cflag |= CSTOPB;//CSTOPB：使用2位停止位
                break;
            default: throw_with_trace(std::runtime_error, "unkown stopbit.")
        }
        tcsetattr(_handle, TCSANOW, &options);

    }

    ~Serial() { close(_handle); }


    bool Send(void *data, int dataLenth) {
        if (_synState) {
            DWORD sended;

            if (!write(_handle, data, dataLenth))
                return false;

            LOG(INFO) << "sended " << dataLenth << " bits";
            return dataLenth == sended;
        } else throw_with_trace(std::runtime_error, "Undefined synState");
    }

    DWORD Recv(BYTE *data, int dataMaxLenth) {
        if (_synState) {
            DWORD received;

            if (!read(_handle, data, dataMaxLenth))
                return 0;

            return received;
        } else throw_with_trace(std::runtime_error, "Undefined synState");
        return 0;
    }
};



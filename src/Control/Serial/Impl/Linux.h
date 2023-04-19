//
// Created by soulde on 2023/4/5.
//
#pragma once

class Serial {

protected:
    int _handle;

public:
    Serial() = default;


    Serial(const char *portName, uint32_t baudrate = UGAS_115200, uint8_t parity = NOPARITY, uint8_t databit = 8,
           uint8_t stopbit = ONESTOPBIT, bool synFlag = true) {

        _handle = open(portName, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (_handle >= 0) {
            std::cout << "Serial[" << _handle << "] " << portName << "start success" << std::endl;
        } else {
            throw_with_trace(std::runtime_error, "can not open serial");
        }

        fcntl(_handle, F_SETFL, 0);

        termios options{};
        cfmakeraw(&options);
        options.c_cflag |= (CLOCAL | CREAD);
        cfsetispeed(&options, baudrate);
        cfsetospeed(&options, baudrate);
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
            default: {
                throw_with_trace(std::runtime_error, "unkown databit.");

            }
        }

        switch (stopbit) {
            case 0:
                options.c_cflag &= ~CSTOPB;//CSTOPB：使用1位停止位
                break;
            case 2:
                options.c_cflag |= CSTOPB;//CSTOPB：使用2位停止位
                break;
            default: {
                throw_with_trace(std::runtime_error, "unkown stopbit.");
                return -1;
            }
        }
        tcsetattr(_handle, TCSANOW, &options);
        return 0;
    }

    ~Serial() { close(_handle); }

    uint32_t Send(void *data, int dataLenth) const {
        uint32_t ret = write(_handle, data, dataLenth);
        if (ret <= 0) {
            throw_with_trace(std::runtime_error, "can not send msg successfully");

        } else {
            LOG(INFO) << "sent " << ret << " bits";
        }
        return ret;
    }

    uint32_t Recv(uint8_t *data, int dataMaxLenth) const {
        return read(_handle, data, dataMaxLenth);
    }

};



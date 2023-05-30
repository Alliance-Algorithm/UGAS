/*
Creation Date: 2023/05/28
Latest Update: 2023/05/31
Developer(s): 22-HJB
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
状态查询的串口通讯
*/

#ifndef UGAS_STATUSCHECK_H
#define UGAS_STATUSCHECK_H

#include <thread>
#include <chrono>

#include "Util/Serial/CRC.h"
#include "Util/Serial/SerialUtil.h"
#include "Util/Debug/Log.h"
#include "Util/Parameter/Parameters.h"


#pragma pack(push, 1)

// NUC目前状态
struct NUCInfo {
    uint8_t RGB[3];
    uint16_t tune, sound;
    // 蜂鸣器音调和响度
    // tune为蜂鸣器两次发声间隔，tune越小音调越高，切记不要发送0
};

// 机器人目前状态
struct DataReceive {
    uint8_t status;
};
#pragma pack(pop)
using DataSend = NUCInfo;



class StatusBoard {
public:
    enum class COLOR { RED, GREEN, BLUE, YELLOW, PURPLE, CYAN, WHITE, BLACK };

    StatusBoard(const char* portName, const uint16_t LEDDelay = 1000,const uint16_t SendDelay = 50) :
        _serial(portName, 115200, serial::Timeout::simpleTimeout(0)),
        _sender(_serial),
        _receiver(_serial),
        _LEDDelay(LEDDelay),
        _delay(SendDelay),
        _destructed(false),
        _thread(&StatusBoard::_serialMain, this)
    {
        info.RGB[0] = 0;
        info.RGB[1] = 0;
        info.RGB[2] = 0;
        info.tune = 0xfff;
        info.sound = 0x0000;
    }
    StatusBoard(const StatusBoard&) = delete;
    StatusBoard(StatusBoard&&) = delete;
    ~StatusBoard() {
        _destructed = true;
        _thread.join();
    }

    void SetColor(enum class COLOR type, int priority = 0) {
        if (priority >= _priority)
        {
            switch (type) {
            case COLOR::BLACK:
                info.RGB[0] = 0;
                info.RGB[1] = 0;
                info.RGB[2] = 0;
                break;
            case COLOR::WHITE:
                info.RGB[0] = 255;
                info.RGB[1] = 255;
                info.RGB[2] = 255;
                break;
            case COLOR::CYAN:
                info.RGB[0] = 0;
                info.RGB[1] = 255;
                info.RGB[2] = 255;
                break;
            case COLOR::YELLOW:
                info.RGB[0] = 255;
                info.RGB[1] = 255;
                info.RGB[2] = 0;
                break;
            case COLOR::PURPLE:
                info.RGB[0] = 255;
                info.RGB[1] = 0;
                info.RGB[2] = 255;
                break;
            case COLOR::RED:
                info.RGB[0] = 255;
                info.RGB[1] = 0;
                info.RGB[2] = 0;
                break;
            case COLOR::GREEN:
                info.RGB[0] = 0;
                info.RGB[1] = 255;
                info.RGB[2] = 0;
                break;
            case COLOR::BLUE:
                info.RGB[0] = 0;
                info.RGB[1] = 0;
                info.RGB[2] = 255;
                break;
            }
        }
    }

    void SetColor(const uint8_t R, const uint8_t G, const uint8_t B, int priority = 0) {
        if (priority >= _priority)
        {
            info.RGB[0] = R;
            info.RGB[1] = G;
            info.RGB[2] = B;
            _priority = priority;
        }
    }

    void SetSound(const uint16_t tune, const uint16_t sound, int priority = 0) {
        if (priority >= _priority)
        {
            info.tune = tune;
            info.sound = sound;
            _priority = priority;
        }
    }

    void SetLEDCircleTimes(uint8_t circleTime, int priority = 0) {
        if (priority >= _priority)
        {
            _circleTime = circleTime;
            _priority = priority;
        }
    }

    void SetInfo(enum class COLOR type, const uint16_t tune, const uint16_t sound, uint8_t circleTime, int priority = 0) {
        if (priority >= _priority)
        {
            SetColor(type, priority);
            SetSound(tune, sound, priority);
            SetLEDCircleTimes(circleTime, priority);
            _priority = priority;
        }
    }

    void SetInfo(const uint8_t R, const uint8_t G, const uint8_t B, const uint16_t tune, const uint16_t sound, uint8_t circleTime, int priority = 0) {
        if (priority >= _priority)
        {
            SetColor(R, G, B, priority);
            SetSound(tune, sound, priority);
            SetLEDCircleTimes(circleTime, priority);
            _priority = priority;
        }
    }

    void SetInfo(const NUCInfo& nucInfo, int priority = 0) {
        if (priority >= _priority)
        {
            info.RGB[0] = nucInfo.RGB[0];
            info.RGB[1] = nucInfo.RGB[1];
            info.RGB[2] = nucInfo.RGB[2];
            info.tune = nucInfo.tune;
            info.sound = nucInfo.sound;
            _priority = priority;
        }
    }

    void ResetPriority() {
        _priority = 0;
    }

    uint8_t GetStatus() {
        return _status;
    }

    NUCInfo GetInfo() {
        return info;
    }

private:

    void _serialMain() {
        while (!_destructed) {
            size_t times = _LEDDelay / _delay;
            turnOnLED = false;
            turnOnSound = true;
            for (uint8_t i = 0; i < times * 2; i++)
            {
                Send();
                Receive();
                std::this_thread::sleep_for(std::chrono::milliseconds(_delay));
            }
            turnOnLED = true;
            turnOnSound = false;
            for (uint8_t i = 0; i < _circleTime * 2 - 1; i++) 
            {
                for (uint8_t j = 0; j < times; j++) 
                {
                    Send();
                    Receive();
                    std::this_thread::sleep_for(std::chrono::milliseconds(_delay));
                }
                turnOnLED = !turnOnLED;
            }
        }
    }

    void Send() {
        NUCInfo data = info;

        if (!turnOnLED)
        {
            data.RGB[0] = 0;
            data.RGB[1] = 0;
            data.RGB[2] = 0;
        }
        if (!turnOnSound) {
            data.sound = 0;
        }
        _sender.Data = data;
        _sender.Send();
    }

    void Receive() {
        bool received = false;
        while (true) {
            auto result = _receiver.Receive();
            if (result == SerialUtil::ReceiveResult::Success)
                received = true;
            else if (result == SerialUtil::ReceiveResult::Timeout)
                break;
            else if (result == SerialUtil::ReceiveResult::InvaildHeader)
                LOG(WARNING) << "StatusBoard: Invaild Header!";
            else if (result == SerialUtil::ReceiveResult::InvaildVerifyDegit)
                LOG(WARNING) << "StatusBoard: Invaild Verify Degit!";
        }
        if (received) {
            const auto& data = _receiver.GetReceivedData();
            _status = data.status;
        }
    }

    bool _destructed;
    serial::Serial _serial;
    SerialUtil::SerialSender<DataSend, SerialUtil::Head<uint8_t, 0xf4>, CRC::DjiCRC8Calculator> _sender;
    SerialUtil::SerialReceiver<DataReceive, SerialUtil::Head<uint8_t, 0xf3>, CRC::DjiCRC8Calculator> _receiver;

    std::thread _thread;

    bool turnOnLED;         // 是否点亮LED
    bool turnOnSound;       // 是否使用蜂鸣器
    NUCInfo info;           // NUC状态
    uint8_t _status = 0;    // 机器人姿态
    uint8_t _priority = 0;  // 当前任务优先级
    uint8_t _circleTime;    // LED循环次数
    uint16_t _LEDDelay;     // LED亮灭间隔(毫秒)
    uint16_t _delay;        // 向单片机发送消息间隔(毫秒)
};

#endif

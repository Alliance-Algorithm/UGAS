#pragma once
/*
Creation Date: 2023/06/03
Latest Update: 2023/06/03
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
与哨兵的串口通讯
*/

#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>

#include "Control/Gimbal/Gimbal.h"
#include "Util/Serial/CRC.h"
#include "Util/Serial/SerialUtil.h"
#include "Util/Debug/Log.h"
#include "Util/Parameter/Parameters.h"

class CBoardSentry {
public:

#pragma pack(push, 1)
    struct DataSend {
        float pitch0, yaw0;
        float pitch1, yaw1;
        uint8_t fire0, fire1;
    };
    struct DataReceive {
        uint8_t selfColor;             // 自身队伍颜色：1-红，2-蓝
        uint8_t presetBulletSpeed;     // 预设弹速，单位：m/s
        float bulletSpeed;             // 实时弹速，单位：m/s
        uint8_t aimEngine;
        uint8_t aimSentry;
        uint8_t aimOnlyOutpost;
    };
#pragma pack(pop)

    CBoardSentry(const char* portName) :
        _serial(portName, 115200, serial::Timeout::simpleTimeout(0)),
        _sender(_serial),
        _receiver(_serial) {
    }

    ~CBoardSentry() { }

    /*! 向哨兵发送云台瞄准数据
    * \param yaw pitch 单位使用弧度制，方向遵循右手定则
    */
    void Send(double yaw, double pitch) {
        _sender.Data.yaw0 = 0;
        _sender.Data.pitch0 = 0;
        _sender.Data.fire0 = 0;

        _sender.Data.yaw1 = -yaw * 180.0 / MathConsts::Pi;
        _sender.Data.pitch1 = -pitch * 180.0 / MathConsts::Pi;
        _sender.Data.fire1 = 0;

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
                LOG(WARNING) << "CboardInfantry: Invaild Header!";
            else if (result == SerialUtil::ReceiveResult::InvaildVerifyDegit)
                LOG(WARNING) << "CboardInfantry: Invaild Verify Degit!";
        }

        if (received) {
            const auto& data = _receiver.GetReceivedData();

            if (data.selfColor == 1)        // 己方红色，击打蓝色
                _enemyColor = ArmorColor::Blue;
            else if (data.selfColor == 2)   // 己方蓝色，击打红色
                _enemyColor = ArmorColor::Red;

            _bulletSpeed = data.presetBulletSpeed;  // 暂时不处理实时弹速
        }
    }

    ArmorColor GetEnemyColor() {
        return _enemyColor;
    }

    float GetBulletSpeed() {
        return _bulletSpeed;
    }

private:
    serial::Serial _serial;

    SerialUtil::SerialSender<DataSend, SerialUtil::Head<uint8_t, 0xff>, CRC::DjiCRC8Calculator> _sender;
    SerialUtil::SerialReceiver<DataReceive, SerialUtil::Head<uint8_t, 0xff>, CRC::DjiCRC8Calculator> _receiver;
    ArmorColor _enemyColor = Parameters::DefaultEnemyColor;
    float _bulletSpeed = Parameters::DefaultBulletSpeed;

};

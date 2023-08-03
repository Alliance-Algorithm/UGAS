#pragma once
/*
Creation Date: 2023/08/02
Latest Update: 2023/08/02
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
与无人机的串口通讯
*/

#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>

#include "Control/Gimbal/GimbalInfantry.h"
#include "Util/Serial/CRC.h"
#include "Util/Serial/SerialUtil.h"
#include "Util/Debug/Log.h"
#include "Util/Parameter/Parameters.h"

class CBoardUav {
public:

#pragma pack(push, 1)
    struct DataSend {
        float yaw, pitch;
    };
    struct DataReceive {
        uint8_t self_color;              // 自身队伍颜色：1-红，2-蓝
        uint8_t preset_bullet_speed;     // 预设弹速，单位：m/s
        float bullet_speed;              // 实时弹速，单位：m/s
        uint8_t auto_scope_enabled;      // 操作手是否开启自瞄
        uint8_t attack_sentry_enabled;   // 是否允许攻击哨兵
        // uint8_t attack_outpost_enabled;  // 是否允许攻击前哨站
    };
#pragma pack(pop)

    explicit CBoardUav(const char* portName) :
        serial_(portName, 115200, serial::Timeout::simpleTimeout(0)),
        _sender(serial_),
        _receiver(serial_) {
    }

    ~CBoardUav() = default;

    // 向无人机发送云台瞄准数据
    // 没有目标时调用
    void Send() {
        _sender.Data.yaw = 0.0f;
        _sender.Data.pitch = 0.0f;
        _sender.Send();
    }

    // 向无人机发送云台瞄准数据
    // yaw pitch: 击中目标所需云台移动差值，单位使用弧度制，方向遵循右手定则
    // fire: 保留
    void Send(double yaw, double pitch, bool fire) {
        _sender.Data.yaw = static_cast<float>(-yaw);
        _sender.Data.pitch = static_cast<float>(-pitch);
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
                LOG(WARNING) << "CBoardUav: Invaild Header!";
            else if (result == SerialUtil::ReceiveResult::InvaildVerifyDegit)
                LOG(WARNING) << "CBoardUav: Invaild Verify Degit!";
        }

        if (received) {
            if (!receive_succeed_) {
                LOG(INFO) << "CBoardUav: Successfully received package.";
                receive_succeed_ = true;
            }

            const auto& data = _receiver.GetReceivedData();

            if (data.self_color == 1)        // 己方红色，击打蓝色
                enemy_color_ = ArmorColor::Blue;
            else if (data.self_color == 2)   // 己方蓝色，击打红色
                enemy_color_ = ArmorColor::Red;

            auto_scope_enabled_ = data.auto_scope_enabled;
            attack_sentry_enabled_ = data.attack_sentry_enabled;
            // attack_outpost_enabled_ = data.attack_outpost_enabled;
        }
    }

    [[nodiscard]] ArmorColor get_enemy_color() const {
        return enemy_color_;
    }

    [[nodiscard]] double get_bullet_speed() const {
        return bullet_speed_;
    }

    [[nodiscard]] bool get_auto_scope_enabled() const {
        return auto_scope_enabled_;
    }

    [[nodiscard]] bool get_attack_sentry_enabled() const {
        return attack_sentry_enabled_;
    }

    [[nodiscard]] bool get_attack_outpost_enabled() const {
        return attack_outpost_enabled_;
    }

private:
    serial::Serial serial_;
    bool receive_succeed_ = false;

    SerialUtil::SerialSender<DataSend, SerialUtil::Head<uint8_t, 0xff>, CRC::DjiCRC8Calculator> _sender;
    SerialUtil::SerialReceiver<DataReceive, SerialUtil::Head<uint8_t, 0xff>, CRC::DjiCRC8Calculator> _receiver;
    ArmorColor enemy_color_ = parameters::DefaultEnemyColor;
    double bullet_speed_ = parameters::AverageBulletSpeed30;
    bool auto_scope_enabled_ = false;
    bool attack_sentry_enabled_ = false, attack_outpost_enabled_ = false;
};

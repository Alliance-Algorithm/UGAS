#pragma once
/*
Creation Date: 2023/04/21
Latest Update: 2023/08/02
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
与步兵的串口通讯
*/

#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>

#include "Util/Serial/CRC.h"
#include "Util/Serial/SerialUtil.h"
#include "Util/Debug/Log.h"
#include "Util/Parameter/Parameters.h"

class CBoardInfantry {
public:

#pragma pack(push, 1)
    struct DataSend {
        float yaw, pitch;
        uint16_t rect_x, rect_y;
        uint8_t color;
    };
    struct DataReceive {
        uint8_t self_color;              // 自身队伍颜色：1-红，2-蓝
        uint8_t preset_bullet_speed;     // 预设弹速，单位：m/s
        float bullet_speed;              // 实时弹速，单位：m/s
        uint8_t auto_scope_enabled;      // 操作手是否开启自瞄
    };
#pragma pack(pop)

    explicit CBoardInfantry(const char* portName) :
            serial_(portName, 115200, serial::Timeout::simpleTimeout(0)),
            sender_(serial_),
            receiver_(serial_) {
    }

    ~CBoardInfantry() = default;

    // 向除哨兵外的地面兵种发送云台瞄准数据
    // 没有目标时调用
    void Send() {
        sender_.Data.yaw = 0.0f;
        sender_.Data.pitch = 0.0f;
        sender_.Send();
    }

    /*! 向除哨兵外的地面兵种发送云台瞄准数据
    * yaw pitch: 击中目标所需云台移动差值，单位使用弧度制，方向遵循右手定则
    */
    void Send(double yaw, double pitch, uint16_t rect_x, uint16_t rect_y) {
        sender_.Data.yaw = static_cast<float>(-yaw * 180.0 / parameters::Pi);
        sender_.Data.pitch = static_cast<float>(-pitch * 180.0 / parameters::Pi);
        sender_.Data.rect_x = 960;
        sender_.Data.rect_y = 540;
        sender_.Data.color = 1;
        sender_.Send();
    }

//    /*! 向无人机发送云台瞄准数据
//    * \param yaw pitch 单位使用弧度制，方向遵循右手定则
//    */
//    void SendUAV(double yaw, double pitch) {
//        _sender.Data.yaw = -yaw;
//        _sender.Data.pitch = -pitch;
//        _sender.Send();
//    }
    
    void Receive() {
        bool received = false;

        while (true) {
            auto result = receiver_.Receive();
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
            if (!receive_succeed_) {
                LOG(INFO) << "CboardInfantry: Successfully received package.";
                receive_succeed_ = true;
            }

            const auto& data = receiver_.GetReceivedData();

            if (data.self_color == 1)        // 己方红色，击打蓝色
                enemy_color_ = ArmorColor::Blue;
            else if (data.self_color == 2)   // 己方蓝色，击打红色
                enemy_color_ = ArmorColor::Red;

            switch (data.preset_bullet_speed) {
                case 26: // 30m/s max
                    bullet_speed_ = parameters::AverageBulletSpeed30;
                    break;
                case 16: // 18m/s max
                    bullet_speed_ = parameters::AverageBulletSpeed18;
                    break;
                case 13: // 15m/s max || 22m/s max?
                    bullet_speed_ = parameters::AverageBulletSpeed15;
                    break;
            }

            auto_scope_enabled_ = data.auto_scope_enabled;
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

private:
    serial::Serial serial_;
    bool receive_succeed_ = false;

    SerialUtil::SerialSender<DataSend, SerialUtil::Head<uint8_t, 0xff>, CRC::DjiCRC8Calculator> sender_;
    SerialUtil::SerialReceiver<DataReceive, SerialUtil::Head<uint8_t, 0xff>, CRC::DjiCRC8Calculator> receiver_;
    ArmorColor enemy_color_ = parameters::DefaultEnemyColor;
    double bullet_speed_ = parameters::DefaultBulletSpeed;
    bool auto_scope_enabled_ = false;
};

#pragma once
/*
Creation Date: 2023/08/09
Latest Update: 2023/08/09
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 步兵云台控制（多线程）
*/

#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>

#include "Control/Serial/HiPNUC.h"
#include "Core/Tracker/TrackerStruct.h"
#include "Util/Serial/CRC.h"
#include "Util/Serial/SerialUtil.h"
#include "Util/Debug/Log.h"
#include "Util/Parameter/Parameters.h"

class CBoardInfantryAsync {
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
        uint8_t buff_mode_enabled;       // 打符模式是否开启
    };
#pragma pack(pop)

    explicit CBoardInfantryAsync(std::string imu_port, const std::string& cboard_port) :
            imu_(std::move(imu_port)),
            serial_(cboard_port, 115200, serial::Timeout::simpleTimeout(0)),
            sender_(serial_),
            receiver_(serial_),
            thread_(&CBoardInfantryAsync::ThreadMain, this) {
        serial_.flush();
    }

    ~CBoardInfantryAsync() {
        destructed_ = true;
        if (thread_.joinable())
            thread_.join();
    };

    void Send(std::unique_ptr<TargetInterface> target, const std::chrono::steady_clock::time_point& timestamp) {
        auto guard = std::lock_guard(target_mutex_);
        target_.swap(target);
        last_update_ = timestamp;
    }

    // 没有目标时调用
    void Send() {
        auto guard = std::lock_guard(target_mutex_);
        target_ = nullptr;
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

    [[nodiscard]] bool get_buff_mode_enabled() const {
        return buff_mode_enabled_;
    }

private:
    void ThreadMain() {
        while (!destructed_) {
            if (imu_.Receive()) {
                SendInternal();
                ReceiveInternal();
                if (fps_.Count()) {
                    std::cout << "CBoardAsync Fps: " << fps_.GetFPS() << '\n';
                }
            }
        }
    }

    void SendInternal() {
        auto timestamp = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(timestamp - last_update_).count();
        double yaw, pitch;
        GimbalGyro::Position now_target_position;
        bool target_available = false;
        {
            auto guard = std::lock_guard(target_mutex_);
            if (target_) {
                now_target_position = target_->Predict(dt);
                auto&& aim = Trajectory_V1::GetShotAngle(*target_, get_bullet_speed(), true, dt);
                yaw = std::get<0>(aim);
                pitch = std::get<1>(aim);
                target_available = true;
            }
        }
        if (target_available) {
            auto [rect_x, rect_y] = ArmorPnPSolver::ReProjection(now_target_position);
            yaw += parameters::StaticYawOffset;
            pitch += parameters::StaticPitchOffset;
            SerialSend(yaw, pitch, rect_x, rect_y);
        }
        else SerialSend();
    }

    void SerialSend() {
        sender_.Data.yaw = 0.0f;
        sender_.Data.pitch = 0.0f;
        sender_.Send();
    }

    void SerialSend(double yaw, double pitch, uint16_t rect_x, uint16_t rect_y) {
        sender_.Data.yaw = static_cast<float>(-yaw * 180.0 / parameters::Pi);
        sender_.Data.pitch = static_cast<float>(-pitch * 180.0 / parameters::Pi);
        sender_.Data.rect_x = rect_x;
        sender_.Data.rect_y = rect_y;
        sender_.Data.color = 1;
        sender_.Send();
    }

    void ReceiveInternal() {
        bool received = false;

        while (true) {
            auto result = receiver_.Receive();
            if (result == SerialUtil::ReceiveResult::Success)
                received = true;
            else if (result == SerialUtil::ReceiveResult::Timeout)
                break;
            else if (result == SerialUtil::ReceiveResult::InvaildHeader)
                LOG(WARNING) << "CBoardInfantryAsync: Invaild Header!";
            else if (result == SerialUtil::ReceiveResult::InvaildVerifyDegit)
                LOG(WARNING) << "CBoardInfantryAsync: Invaild Verify Degit!";
        }

        if (received) {
            if (!receive_succeed_) {
                LOG(INFO) << "CBoardInfantryAsync: Successfully received package.";
                receive_succeed_ = true;
            }

            const auto& data = receiver_.GetReceivedData();

            if (data.self_color == 1)        // 己方红色，击打蓝色
                enemy_color_ = ArmorColor::Blue;
            else if (data.self_color == 2)   // 己方蓝色，击打红色
                enemy_color_ = ArmorColor::Red;

            switch (data.preset_bullet_speed) {
                case 30: case 26: // 30m/s max
                    bullet_speed_ = parameters::AverageBulletSpeed30;
                    break;
                case 18: case 16: // 18m/s max
                    bullet_speed_ = parameters::AverageBulletSpeed18;
                    break;
                case 15: case 13: // 15m/s max
                    bullet_speed_ = parameters::AverageBulletSpeed15;
                    break;
            }

            auto_scope_enabled_ = data.auto_scope_enabled;
            buff_mode_enabled_ = data.buff_mode_enabled;
        }
    }

    HiPNUC imu_;
    serial::Serial serial_;
    FPSCounter_V2 fps_;

    SerialUtil::SerialSender<DataSend, SerialUtil::Head<uint8_t, 0xff>, CRC::DjiCRC8Calculator> sender_;
    SerialUtil::SerialReceiver<DataReceive, SerialUtil::Head<uint8_t, 0xff>, CRC::DjiCRC8Calculator> receiver_;

    std::atomic<bool> receive_succeed_ = false;
    std::atomic<ArmorColor> enemy_color_ = parameters::DefaultEnemyColor;
    std::atomic<double> bullet_speed_ = parameters::DefaultBulletSpeed;
    std::atomic<bool> auto_scope_enabled_ = false, buff_mode_enabled_ = false;

    std::unique_ptr<TargetInterface> target_;
    std::chrono::steady_clock::time_point last_update_;
    std::mutex target_mutex_;

    std::thread thread_;
    std::atomic<bool> destructed_ = false;
};

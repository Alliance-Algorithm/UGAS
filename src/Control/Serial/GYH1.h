#pragma once
/*
Creation Date: 2023/04/18
Latest Update: 2023/04/21
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
与开源陀螺仪GY-H1的串口通讯
*/

#include <chrono>
#include <thread>

#include <eigen3/Eigen/Dense>

#include "Control/Gimbal/Gimbal.h"
#include "Util/Serial/CRC.h"
#include "Util/Serial/SerialUtil.h"
#include "Util/Debug/Log.h"

class GYH1 {
public:

#pragma pack(push, 1)
    struct QuaternionReceive {
        char head = 0x00;            // Should be 0x40
        float w, x, y, z;            // Quaternion Data
        char crc8;                    // CRC8
    };
#pragma pack(pop)

#pragma pack(push, 1)
    struct QuaternionData {
        float w, x, y, z;
    };
#pragma pack(pop)

    GYH1(const char* portName) :
        //_serial(portName, 921600, serial::Timeout::simpleTimeout(0)),
        _serial(portName, 864000, serial::Timeout::simpleTimeout(0)),
        _thread(&GYH1::_serialMain, this) {
    }

    ~GYH1() {
        _destructed = true;
        _thread.join();
    }

    Eigen::Quaternionf ReceiveQuaternion() {
        while (true) {
            QuaternionReceive pkg{};
            do {
                _serial.read(reinterpret_cast<uint8_t*>(&pkg), 1);
            } while (pkg.head != 0x40);
            _serial.read(reinterpret_cast<uint8_t*>(&pkg) + 1, sizeof(decltype(pkg)) - 1);
            if (CRC::GyH1CRC8Calculator::Verify(pkg)) {
                //std::cout << pkg.w << '\t' << pkg.x << '\t' << pkg.y << '\t' << pkg.z << '\n';
                return Eigen::Quaternionf(pkg.w, pkg.x, pkg.y, pkg.z);
            }
            else LOG(WARNING) << "ReceiveQuaternion: Invaild CRC8.";
        }
    }

    Eigen::Vector3f ReceiveEularAngle() {
        auto q = ReceiveQuaternion();
        auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
        // "Euler from quaternion in roll, pitch, yaw" 
        std::cout << euler.x() * 180 / MathConsts::Pi << ' ' << euler.y() * 180 / MathConsts::Pi << ' ' << euler.z() * 180 / MathConsts::Pi << '\n';
        return euler;
    }

    std::optional<Eigen::Quaternionf> ReceiveQuaternionNonBlocking() {
        QuaternionReceive pkg{};
        std::optional<Eigen::Quaternionf> result{};
        auto bufferSize = _serial.available();
        while (bufferSize >= sizeof(decltype(pkg))) {
            --bufferSize;
            _serial.read(reinterpret_cast<uint8_t*>(&pkg), 1);
            if (pkg.head == 0x40) {
                _serial.read(reinterpret_cast<uint8_t*>(&pkg) + 1, sizeof(decltype(pkg)) - 1);
                bufferSize -= sizeof(decltype(pkg)) - 1;
                if (CRC::GyH1CRC8Calculator::Verify(pkg)) {
                    result = Eigen::Quaternionf(pkg.w, pkg.x, pkg.y, pkg.z);
                }
                else LOG(WARNING) << "ReceiveQuaternion: Invaild CRC8.";
            }
        }
        return result;
    }

private:

    std::atomic<bool> _destructed = false;
    serial::Serial _serial;
    std::thread _thread;

    void _serialMain() {
        SerialUtil::SerialReceiver<QuaternionData, SerialUtil::Head<uint8_t, 0x40>, CRC::GyH1CRC8Calculator> receiver{ _serial };

        //SerialUtil::SerialReceiver<QuaternionData, SerialUtil::Head<uint8_t, 0x40>, SerialUtil::None> receiver{ _serial };
        
        int failedCount = 0;
        while (!_destructed) {
            int i = 4;
            while (i--) {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                std::cout << _serial.available() << '\n';
            }

            bool received = false;
            while (true) {
                auto result = receiver.Receive();
                if (result == SerialUtil::ReceiveResult::Success)
                    received = true;
                else if (result == SerialUtil::ReceiveResult::Timeout)
                    break;
                else if (result == SerialUtil::ReceiveResult::InvaildHeader)
                    LOG(WARNING) << "Invaild Header!";
                else if (result == SerialUtil::ReceiveResult::InvaildVerifyDegit)
                    LOG(WARNING) << "Invaild Verify Degit!";
            }
            if (received) {
                const auto& data = receiver.GetReceivedData();
                auto q = Eigen::Quaternionf(data.w, data.x, data.y, data.z);
                auto size = _serial.available();
                std::cout << size << ' ' << q << '\n';
            }
            //std::this_thread::sleep_for(std::chrono::milliseconds(100));

        }
    }
};
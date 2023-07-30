#pragma once
/*
Creation Date: 2023/05/05
Latest Update: 2023/05/26
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
超核电子（HiPNUC）的惯性导航设备通讯
*/

#include <chrono>
#include <thread>
#include <chrono>
#include <atomic>
#include <optional>

#include <eigen3/Eigen/Dense>

#include "Util/Serial/SerialUtil.h"
#include "Util/FPSCounter/FPSCounter.h"
#include "Core/Transformer/Tree.h"
#include "Util/ROS/TfBroadcast.h"

class HiPNUC {
public:
    explicit HiPNUC(const char* portName) :
        _portName(portName),
        _destructed(false),
        _thread(&HiPNUC::_serialMain, this) {
    }
    HiPNUC(const HiPNUC&) = delete;
    HiPNUC(HiPNUC&&) = delete;

    ~HiPNUC() {
        _destructed = true;
        _thread.join();
    }

    bool UpdateTransformer() {
        if (_available) {
            const float* quatPtr = _transQuat;
            transformer::SetRotation<GimbalGyro, GimbalLink>(Eigen::Quaterniond{ quatPtr[0], quatPtr[1], quatPtr[2], quatPtr[3] });
        }
        ros_util::TfBroadcast<GimbalGyro, CameraLink>();
        ros_util::TfBroadcast<GimbalGyro, MuzzleLink>();
        return _available;
    }

private:
#pragma pack(push, 1)
    // 标准数据结构体
    struct Data91 {           // Unit      Name
        uint16_t length;      //       帧中数据域的长度，固定值76
        uint16_t crc;         //       除自身外其余所有字段的CRC-16校验和
        uint8_t tag;          //       数据包标签：0x91
        uint16_t useless;     //       保留
        uint8_t avg_temp;     //  °C   模块陀螺仪平均温度
        float pressure;       //  Pa   气压(部分型号支持)
        uint32_t timestamp;   //  ms   开机开始累加的本地时间戳信息，每毫秒增加1
        float acc[3];         //  1G   经过出厂校准后的加速度，顺序为：xyz
        float gyr[3];         // deg/s 经过出厂校准后的角速度，顺序为：xyz
        float mag[3];         //  uT   磁强度，顺序为：xyz
        float eul[3];         //  deg  欧拉角，顺序为：roll, pitch, yaw (yxz)
        float quat[4];        //       节点四元数集合，顺序为：wxyz
    };

    // 只有四元数的数据结构体
    struct DataD1 {           // Unit      Name
        uint16_t length;      //       帧中数据域的长度，固定值17
        uint16_t crc;         //       除自身外其余所有字段的CRC-16校验和
        uint8_t tag1;         //       数据包标签：0xD1
        float quat[4];        //       节点四元数集合，顺序为：wxyz
    };
#pragma pack(pop)


    static constexpr int a = sizeof(Eigen::Quaternionf);

    class DataCRC16Calculator {
    public:
        using ResultType = SerialUtil::None;

        template <typename T>
        static bool Verify(const T& package) {
            static_assert(sizeof(T) == 82 || sizeof(T) == 23, "Wrong package size!");

            uint16_t checksum = 0x00;
            auto src = reinterpret_cast<const uint8_t*>(&package);

            _crc16Update(checksum, src, 4);
            _crc16Update(checksum, src + 6, sizeof(T) - 6);

            return checksum == *reinterpret_cast<const uint16_t*>(src + 4);
        }

    private:
        static void _crc16Update(uint16_t& currectCrc, const uint8_t* src, size_t lengthInBytes) {
            uint32_t crc = currectCrc;
            for (size_t j = 0; j < lengthInBytes; ++j) {
                uint32_t i;
                uint32_t byte = src[j];
                crc ^= byte << 8;
                for (i = 0; i < 8; ++i) {
                    uint32_t temp = crc << 1;
                    if (crc & 0x8000) {
                        temp ^= 0x1021;
                    }
                    crc = temp;
                }
            }
            currectCrc = crc;
        }
    };

    void _serialMain() {
        FPSCounter_V2 imuFps, errorFps;

        while (!_destructed) {
            try {

                serial::Serial serial(_portName, 115200, serial::Timeout::simpleTimeout(100));
                SerialUtil::SerialReceiver<DataD1, SerialUtil::Head<uint16_t, 0xa55a>, DataCRC16Calculator> receiver(serial);

                while (true) {
                    if (_destructed) return;
                    try {

                        auto result = receiver.Receive();
                        if (result == SerialUtil::ReceiveResult::Success) {
                            const auto& data = receiver.GetReceivedData();

                            // 每次GetReceivedData得到的数据，其生命周期持续到下次Receive成功后，再次调用Receive前。
                            // TODO: 这里使用指针存储四元数实现无锁，有极小概率在读取时获得错误数据。
                            _transQuat = &(data.quat[0]);
                            //std::cout << data.quat[0] << ' ' << data.quat[1] << ' ' << data.quat[2] << ' ' << data.quat[3] << '\n';

                            if (imuFps.Count()) {
                                _available = imuFps.GetFPS() > 100;
                                std::cout << "HiPNUC IMU Fps: " << imuFps.GetFPS() << '\n';
                            }
                        }
                        else if (result == SerialUtil::ReceiveResult::InvaildHeader)
                            LOG(WARNING) << "HiPNUC: Invaild Header!";
                        else if (result == SerialUtil::ReceiveResult::InvaildVerifyDegit)
                            LOG(WARNING) << "HiPNUC: Invaild Verify Degit!";
                    }
                    catch (serial::IOException& e) {
                        // windows下的串口会偶发性的抛出IOException，大部分时候可以忽略此异常
                        LOG(ERROR) << "HiPNUC: Unexpected serial::IOException: " << e.what();
                        errorFps.Count();
                        if (errorFps.GetFPS() > 10) throw std::runtime_error("HiPNUC: Exceptions are thrown too frequently.");
                    }
                    catch (serial::SerialException& e) {
                        // linux下的串口会抛出SerialException，一般在串口断开连接时弹出，暂时不清楚会不会偶发性抛出，优先选择忽略此异常
                        LOG(ERROR) << "HiPNUC: Unexpected serial::SerialException: " << e.what();
                        errorFps.Count();
                        if (errorFps.GetFPS() > 10) throw std::runtime_error("HiPNUC: Exceptions are thrown too frequently.");
                    }
                }
                
            }
            catch (serial::IOException& e) {
                LOG(ERROR) << "HiPNUC: Caught serial::IOException when serial init: " << e.what();
            }
            catch (std::exception& e) {
                LOG(ERROR) << "HiPNUC: Uncaught " << typeid(e).name() << ": " << e.what();
            }

            _available = false;
            LOG(ERROR) << "HiPNUC: Will reconnect in 1 second.";
            auto timingStart = std::chrono::steady_clock::now();
            while (std::chrono::steady_clock::now() - timingStart < std::chrono::seconds(1)) {
                if (_destructed) return;
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    }

    const char* _portName;
    std::atomic<bool> _destructed;
    std::thread _thread;

    std::atomic<bool> _available = false;

    const float _defaultTransQuat[4] = { 1, 0, 0, 0 };
    std::atomic<const float*> _transQuat = _defaultTransQuat;
};

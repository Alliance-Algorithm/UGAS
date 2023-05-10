#pragma once
/*
Creation Date: 2023/05/05
Latest Update: 2023/05/05
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
超核电子（HiPNUC）的惯性导航设备通讯
*/

#include <chrono>
#include <thread>

#include <Eigen/Dense>

#include "Util/Serial/SerialUtil.h"
#include "Util/FPSCounter/FPSCounter.h"

class HiPNUC {
public:
	HiPNUC(const char* portName) :
		_serial(portName, 115200, serial::Timeout::simpleTimeout(1000)),
		_receiver(_serial),
		_destructed(false),
		_thread(&HiPNUC::_serialMain, this) {
	}
	HiPNUC(const HiPNUC&) = delete;
	HiPNUC(HiPNUC&&) = delete;

	~HiPNUC() {
		_destructed = true;
		_thread.join();
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
				LOG(WARNING) << "HiPNUC: Invaild Header!";
			else if (result == SerialUtil::ReceiveResult::InvaildVerifyDegit)
				LOG(WARNING) << "HiPNUC: Invaild Verify Degit!";
		}
		if (received) {

		}
	}

	Eigen::Vector3f Transform(const Eigen::Vector3f& pos) const {
		const float* quatPtr = _transQuat;
		auto q = Eigen::Quaternionf{ quatPtr[0], quatPtr[1], quatPtr[2], quatPtr[3] };
		auto tranPos = q * Eigen::Quaternionf{0, pos.x() + 118.05f, -(pos.y() - 67.5f), pos.z() - 41.7f} * q.inverse();
		return { tranPos.x(), -tranPos.y(), tranPos.z() };
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
		double vx = 0, vy = 0, vz = 0;
		double x = 0, y = 0, z = 0;
		int interval = 100;

		auto fps = FPSCounter_V2();

		while (!_destructed) {
			auto result = _receiver.Receive();
			if (result == SerialUtil::ReceiveResult::Success) {
				const auto& data = _receiver.GetReceivedData();

				// 每次GetReceivedData得到的数据，其生命周期持续到下次Receive成功后，再次调用Receive前。
				_transQuat = &(data.quat[0]);

				if (fps.Count()) {
					std::cout << "HiPNUC IMU Fps: " << fps.GetFPS() << '\n';
				}

				//auto q = Eigen::Quaterniond{ data.quat[0], data.quat[1], data.quat[2], data.quat[3] }.normalized();

				//auto euler = q.toRotationMatrix().eulerAngles(2, 0, 1);
				//float q2eul[3] = { euler.z() * 180 / MathConsts::Pi, euler.y() * 180 / MathConsts::Pi, euler.x() * 180 / MathConsts::Pi };
				/*auto acc = q * Eigen::Quaterniond{ 0, data.acc[0], data.acc[1], data.acc[2] } *q.inverse();
				double ax = acc.x(), ay = acc.y(), az = acc.z();
				if (ax < 0.1) ax = 0;
				if (ay < 0.1) ay = 0;
				if (az < 0.1) az = 0;

				vx += ax * MathConsts::G * 0.002, vy += ay * MathConsts::G * 0.002, vz += az * MathConsts::G * 0.002;

				x += vx * 0.002, y += vy * 0.002, z += vz * 0.002;

				if (abs(ay) > 0.1) {
					std::cout << ax << ' ' << ay << ' ' << az << '\n';
				}*/

			}
			else if (result == SerialUtil::ReceiveResult::InvaildHeader)
				LOG(WARNING) << "HiPNUC: Invaild Header!";
			else if (result == SerialUtil::ReceiveResult::InvaildVerifyDegit)
				LOG(WARNING) << "HiPNUC: Invaild Verify Degit!";
		}
	}

	serial::Serial _serial;
	SerialUtil::SerialReceiver<DataD1, SerialUtil::Head<uint16_t, 0xa55a>, DataCRC16Calculator> _receiver;
	std::atomic<bool> _destructed;
	std::thread _thread;

	const float _defaultTransQuat[4] = {1, 0, 0, 0};
	std::atomic<const float*> _transQuat = _defaultTransQuat;
};
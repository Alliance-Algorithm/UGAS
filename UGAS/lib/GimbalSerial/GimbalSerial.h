#pragma once
/*
Creation Date: 2022/10/11
Latest Update: 2022/10/14
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved

Class property:
- 继承自 SendPkg 和 RecvPkg，可直接类型转换使用

Class public functions:
- Open		(const char* portName):
	打开名为portName的串口
- IsOpen	()
	检查串口是否可用，可用true，不可用false
- Close		()
	关闭串口

- Send			()
	发送自身 SendPkg 部分数据
- RecvGimbalData()
	接收数据到自身 RecvPkg 部分
- GetGimbalData	()
	返回自身 RecvPkg 部分数据

- DebugGimbalInfo(std::ostream& os)
	调用 SendPkg 和 RecvPkg Debug输出函数
*/
#include <iostream>
#include "../../Parameters/DebugSettings.h"
#include "../../Parameters/Parameters.h"
#include "Packages.h"

namespace serial {
	class GimbalSerial : public SendPkg, public RecvPkg {
	public:
		GimbalSerial()
		{ // 虚拟云台串口数据初始化
#if VIRTUAL_GIBAL == 1
			memset(this, 0, sizeof(*this));
			RecvPkg::head = '\xFF';
			RecvPkg::team  = DEFAULT_TEAM;
			RecvPkg::speed = 30000.0;
			RecvPkg::flag  = STATE_NORMAL;
#endif
		}

		virtual void Open	(const char* portName)	= 0;
		virtual bool IsOpen	()						= 0;
		virtual void Close	()						= 0;

		virtual bool  Send						()	= 0;
		virtual const RecvPkg& RecvGimbalData	()	= 0;
		inline  const RecvPkg& GetGimbalData	() const
			{ return static_cast<const RecvPkg&>(*this); }

		virtual void DebugGimbalInfo(std::ostream& os) const {
			os << "Gimbal SendPkg:" << std::endl;
			this->SendPkg::Debug(os);
			os << "Gimbal RecvPkg:" << std::endl;
			this->RecvPkg::Debug(os);
		}
	};
}

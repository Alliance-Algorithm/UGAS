#pragma once
/*
Creation Date: 2022/10/11
Latest Update: 2022/10/13
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 定义所有调试宏开关
*/

/// debug state switches 调试开关
#define    VIRTUAL_GIBAL 1					// 虚拟云台开关
#define       DEBUG_PARA 1					// 动态调参拖动条窗口
#define   PRETREAT_DEBUG 0					// 显示预处理后的图像
#define      DEBUG_ARMOR 1					// 显示装甲板识别图像


/// static var			 静态变量
#define VIDEO_VAR_TYPE	const char*			// 视频读入类型
#define VIDEO_VAR		"resources/Red.mp4"	// 视频读入
#define NUM_PARA_TYPE	void*				// 数字识别参数类型
#define NUM_PARA		nullptr				// 数字识别参数
#define DEFAULT_TEAM	Blue				// 虚拟云台队伍颜色

/// constant vars		 常值变量
#define SERIAL_PORT		"\\\\.\\COM3"		// 串口逻辑名


/// var(s) Debug func	调试曲线生成宏定义 #



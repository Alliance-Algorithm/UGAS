#pragma once 
/*
Creation Date: 2022/10/11
Latest Update: 2022/10/13
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 定义所有调试宏开关

* 调试宏使用示例 (单变量)
* DEBUG_GRAGH(var)
* 调试宏使用示例 (多变量)
* MAKE_GRAGH_DEFAULT
*     GRAGH_ADD_VAR(var1, COLOR_YELLOW)
*     GRAGH_ADD_VAR(var2, COLOR_BLUE)
* SHOW_GRAGH(Gragh_var1_var2)
*/

/// debug state switches 调试开关
#define VIRTUAL_GIBAL	1					// 虚拟云台开关
#define DEBUG_IMG		1					// 显示调试图像总开关
#define DEBUG_PARA		0					// 动态调参拖动条窗口
#define DEBUG_PRETREAT	0					// 使用预处理后的图像作为调试图像
#define DEBUG_LIGHTBAR	0					// 绘制灯条识别图像
#define DEBUG_ARMOR		1					// 绘制装甲板识别图像
#define DEBUG_ARMOR_NUM 1					// 绘制装甲板数字识别结果
#define DEBUG_TRACK		1					// 绘制跟踪点
#define DEBUG_PREDICT	1					// 绘制预测点
#define DEBUG_ANGLE		0					// 绘制Yaw、Pitch曲线


/// static var			 静态变量
#define VIDEO_VAR_TYPE	const char*			// 视频读入类型
#define VIDEO_VAR		"Blue_3.mp4" 		// 视频读入
#define NUM_PARA_TYPE	const char*			// 数字识别参数类型
#define NUM_PARA		"V1.pb"				// 数字识别参数
#define FILTER_TYPE		PID::PDfilter		// 目标运动滤波器类型


/// debug var			 调试变量
#define DEFAULT_TEAM	Red 				// 虚拟云台队伍颜色
#define NUM_DEFAULT		1					// 数字识别默认返回数字(一般是0)
#define DEFAULT_MEM_NUM 300					// 调试曲线默认最大采样数量


/// constant vars		 常值变量
#define SERIAL_PORT		"\\\\.\\COM3"		// 串口逻辑名
#define MAX_FPS			1000				// 最大帧数限制
#define MAX_CNT			1000000				// 弹道仿真迭代次数限制
#define PI				3.1415926535897		// Π值
#define G				9.8					// 重力常数


/// Time cost analysis	运行时间分析宏定义
#define START_COUNT	{TimeStamp __tsTmp = TimeStampCounter::GetTimeStamp();
#define PRINT_COST	 printf("Took %llu ms | ", \
						TimeStampCounter::GetTimeStamp() - __tsTmp);
#define END_COUNT	}
#define	PRINT_END_COUNT PRINT_COST END_COUNT

#pragma once
/*
Creation Date: 2022/11/16
Latest Update: 2022/11/20
Developer(s): 22-QZH
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 包装HT工业相机的SDK(windows)
- 仅包含摄像头读入
*/
#include "../../ImgCapture.h"

#ifdef _WIN64
#pragma comment(lib, "MVCAMSDK_X64.lib")
#else
#pragma comment(lib, "MVCAMSDK.lib")
#endif
#include "../../../../third-party/HTCameraSDK/Windows/CameraApi.h"


class WindowsHTCameraCapture :public ImgCapture {
private:
	bool initialized = false;

	HANDLE _hDispThread;         //图像抓取线程的句柄
	CameraHandle _hCamera;       //相机句柄，多个相机同时使用时，可以用数组代替	
	tSdkFrameHead _sFrameInfo;   //用于保存当前图像帧的帧头信息
	BYTE* _pFrameBuffer;         //用于将原始图像数据转换为RGB的缓冲区
	 
protected:
	WindowsHTCameraCapture() = default;
	~WindowsHTCameraCapture();

public:
	//always let useLess = nullptr
	virtual void init(void* useLess);
	virtual void read(Img& img);
};
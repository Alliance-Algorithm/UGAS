#include "WindowsHTCameraCapture.h"


WindowsHTCameraCapture::~WindowsHTCameraCapture() {
	CameraUnInit(_hCamera);
	CameraAlignFree(_pFrameBuffer);
}


void WindowsHTCameraCapture::init(void* useLess) {
	if (initialized) {
		throw_with_trace(std::runtime_error, "Already initialized");
	}
	else {
		tSdkCameraDevInfo sCameraList[10];
		INT iCameraNums;
		CameraSdkStatus status;
		tSdkCameraCapbility sCameraInfo;

		//枚举设备，获得设备列表
		iCameraNums = 10;//调用CameraEnumerateDevice前，先设置iCameraNums = 10，表示最多只读取10个设备，如果需要枚举更多的设备，请更改sCameraList数组的大小和iCameraNums的值

		if (CameraEnumerateDevice(sCameraList, &iCameraNums) != CAMERA_STATUS_SUCCESS || iCameraNums == 0)
		{
			throw_with_trace(std::runtime_error, "No camera was found!");
		}

		//该示例中，我们只假设连接了一个相机。因此，只初始化第一个相机。(-1,-1)表示加载上次退出前保存的参数，如果是第一次使用该相机，则加载默认参数.
		//In this demo ,we just init the first camera.
		if ((status = CameraInit(&sCameraList[0], -1, -1, &_hCamera)) != CAMERA_STATUS_SUCCESS)
		{
			LOG(ERROR) << "Error code is " << status << CameraGetErrorString(status);
			throw_with_trace(std::runtime_error, "Failed to init the camera");
		}


		//Get properties description for this camera.
		CameraGetCapability(_hCamera, &sCameraInfo);//"获得该相机的特性描述"

		_pFrameBuffer = (BYTE*)CameraAlignMalloc(sCameraInfo.sResolutionRange.iWidthMax * sCameraInfo.sResolutionRange.iWidthMax * 3, 16);

		if (sCameraInfo.sIspCapacity.bMonoSensor)
		{
			CameraSetIspOutFormat(_hCamera, CAMERA_MEDIA_TYPE_MONO8);
		}

		//strcpy_s(g_CameraName, sCameraList[0].acFriendlyName);

		CameraPlay(_hCamera);

		initialized = true;
	}
}

void WindowsHTCameraCapture::read(Img& img) {
	if (initialized) {
		BYTE* pbyBuffer;
		CameraSdkStatus status;
		if (status = CameraGetImageBuffer(_hCamera, &_sFrameInfo, &pbyBuffer, 1000) == CAMERA_STATUS_SUCCESS)
		{
			//将获得的原始数据转换成RGB格式的数据，同时经过ISP模块，对图像进行降噪，边沿提升，颜色校正等处理。
			//我公司大部分型号的相机，原始数据都是Bayer格式的
			if (status = CameraImageProcess(_hCamera, pbyBuffer, _pFrameBuffer, &_sFrameInfo) == CAMERA_STATUS_SUCCESS) {
				//调用SDK封装好的显示接口来显示图像,您也可以将m_pFrameBuffer中的RGB数据通过其他方式显示，比如directX,OpengGL,等方式。
				CameraImageOverlay(_hCamera, _pFrameBuffer, &_sFrameInfo);

				// 由于SDK输出的数据默认是从底到顶的，转换为Opencv图片需要做一下垂直镜像
				CameraFlipFrameBuffer(_pFrameBuffer, &_sFrameInfo, 1);

				cv::Mat matImage(
					cv::Size(_sFrameInfo.iWidth, _sFrameInfo.iHeight),
					_sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
					_pFrameBuffer
				);
				img = matImage;
			}
			else {
				LOG(ERROR) << "CameraImageProcess status = " << status;
				throw_with_trace(std::runtime_error, "Failed to execute CameraImageProcess");
			}

			//在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
			//否则再次调用CameraGetImageBuffer时，程序将被挂起，直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
			CameraReleaseImageBuffer(_hCamera, pbyBuffer);

			memcpy(&_sFrameInfo, &_sFrameInfo, sizeof(tSdkFrameHead));
		}
		else {
			LOG(ERROR) << "CameraGetImageBuffer status = " << status;
			throw_with_trace(std::runtime_error, "Failed to execute CameraGetImageBuffer");
		}
	}
	else {
		throw_with_trace(std::runtime_error, "Not initialized");
	}
}

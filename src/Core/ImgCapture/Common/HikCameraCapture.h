#pragma once
/*
Creation Date: 2022/11/16
Latest Update: 2022/11/20
Developer(s): 22-QZH 22-Iruma
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 包装海康工业相机的SDK
- 支持处理多个摄像头，需传入UserDefinedName（在MVS中设置）
*/

#include <cstring>

#include <opencv2/opencv.hpp>

#ifdef WIN32
#include "ThirdParty/HikCameraSDK/Windows/MvCameraControl.h"
#else
#include <MvCameraControl.h>
#endif

#include "Core/ImgCapture/ImgCaptureInterface.h"
#include "Util/Exceptions.h"
#include "Util/Debug/Log.h"

class HikCameraCapture : public ImgCaptureInterface {
private:
    void* _handle = nullptr;
    unsigned int ConvertDataSize = 0;
    unsigned char* pConvertData = nullptr;
    MV_CC_PIXEL_CONVERT_PARAM stConvertParam;

    bool IsSameDeviceName(MV_CC_DEVICE_INFO* pstMVDevInfo, const char* targetName) {
        if (nullptr == pstMVDevInfo) {
            LOG(INFO) << "The Pointer of pstMVDevInfo is NULL!";
            return false;
        }
        const unsigned char* deviceName;
        if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE) {
            deviceName = pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName;
        }
        else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE) {
            deviceName = pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName;
        }
        else {
            LOG(WARNING) << "Neither a GigE camera nor a USB camera.";
            return false;
        }

        return strcmp(reinterpret_cast<const char*>(deviceName), targetName) == 0;
    }

    bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo) {
        if (nullptr == pstMVDevInfo) {
            LOG(INFO) << "The Pointer of pstMVDevInfo is NULL!";
            return false;
        }
        if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE) {
            int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
            int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
            int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
            int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);
            LOG(INFO) << "DeviceIp: " << nIp1 << ' ' << nIp2 << ' ' << nIp3 << ' ' << nIp4;
            LOG(INFO) << "UserDefinedName: " << pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName;
        }
        else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE) {
            LOG(INFO) << "UserDefinedName: " << pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName;
            LOG(INFO) << "Serial Number: " << pstMVDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber;
            LOG(INFO) << "Device Number: " << pstMVDevInfo->SpecialInfo.stUsb3VInfo.nDeviceNumber;
        }
        else {
            LOG(WARNING) << "Neither a GigE camera nor a USB camera.";
            return false;
        }

        return true;
    }

    bool InitCamera(const char* userDefinedName) {
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (MV_OK == nRet) {

            MV_CC_DEVICE_INFO* pDeviceInfo = nullptr;
            if (stDeviceList.nDeviceNum == 1) {
                pDeviceInfo = stDeviceList.pDeviceInfo[0];
                if (nullptr != userDefinedName && !IsSameDeviceName(pDeviceInfo, userDefinedName)) {
                    PrintDeviceInfo(pDeviceInfo);
                    LOG(WARNING) << "Name of the only camera does not equal to the name passed in: " << userDefinedName;
                }
            }
            else if (stDeviceList.nDeviceNum > 1)
            {
                if (nullptr != userDefinedName) {
                    for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++) {
                        if (nullptr == pDeviceInfo) break;
                        if (IsSameDeviceName(stDeviceList.pDeviceInfo[i], userDefinedName)) {
                            pDeviceInfo = stDeviceList.pDeviceInfo[i];
                            break;
                        }
                    }
                    if (nullptr == pDeviceInfo) {
                        LOG(ERROR) << stDeviceList.nDeviceNum << " devices was found, but no device matches the name passed in: " << userDefinedName;
                    }
                }
                else LOG(ERROR) << stDeviceList.nDeviceNum << " devices was found, please pass in the device name.";
            }
            else LOG(ERROR) << "Find No Devices.";

            if (nullptr != pDeviceInfo) {

                nRet = MV_CC_CreateHandle(&_handle, pDeviceInfo);
                if (MV_OK == nRet) {

                    nRet = MV_CC_OpenDevice(_handle);
                    if (MV_OK == nRet) {

                        if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE)
                        {
                            int nPacketSize = MV_CC_GetOptimalPacketSize(_handle);
                            if (nPacketSize > 0)
                            {
                                nRet = MV_CC_SetIntValue(_handle, "GevSCPSPacketSize", nPacketSize);
                                if (MV_OK != nRet)
                                    LOG(WARNING) << "Warning: Failed to set packet Size. nRet [" << nRet << "]";
                            }
                            else LOG(WARNING) << "Warning: Get invaild packet Size: " << nPacketSize;
                        }

                        nRet = MV_CC_SetEnumValue(_handle, "TriggerMode", MV_TRIGGER_MODE_OFF);
                        if (MV_OK == nRet) {

                            nRet = MV_CC_SetEnumValue(_handle, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
                            if (MV_OK != nRet) LOG(WARNING) << "Warning: Failed to set auto exposure. nRet [" << nRet << "]";

                            nRet = MV_CC_SetFloatValue(_handle, "ExposureTime", true ? 2000 : 1000);
                            if (MV_OK != nRet) LOG(WARNING) << "Warning: Failed to set exposure time. nRet [" << nRet << "]";

                            nRet = MV_CC_SetFloatValue(_handle, "Gain", true ? 16.9807f : 0.0f);
                            if (MV_OK != nRet) LOG(WARNING) << "Warning: Failed to set gain. nRet [" << nRet << "]";

                            nRet = MV_CC_SetBoolValue(_handle, "DigitalShiftEnable", true);
                            if (MV_OK != nRet) LOG(WARNING) << "Warning: Failed to set digital shift enable. nRet [" << nRet << "]";

                            nRet = MV_CC_SetFloatValue(_handle, "DigitalShift", 5.9993f);
                            if (MV_OK != nRet) LOG(WARNING) << "Warning: Failed to set digital shift. nRet [" << nRet << "]";

                            nRet = MV_CC_SetBoolValue(_handle, "AcquisitionFrameRateEnable", false);
                            if (MV_OK != nRet) LOG(WARNING) << "Warning: Failed to set acquisition frame rate enable. nRet [" << nRet << "]";

                            nRet = MV_CC_StartGrabbing(_handle);
                            if (MV_OK == nRet) {
                                return true;
                            }
                            else LOG(ERROR) << "Failed to start grabbing. nRet [" << nRet << ']';

                        }
                        else LOG(ERROR) << "Failed to set trigger Mode. nRet [" << nRet << ']';
                        MV_CC_CloseDevice(_handle);

                    }
                    else LOG(ERROR) << "Failed to open device. nRet [" << nRet << ']';
                    MV_CC_DestroyHandle(_handle);

                }
                else LOG(ERROR) << "Failed to create handle. nRet [" << nRet << ']';

            }
        }
        else LOG(ERROR) << "Failed to enum Devices. nRet [" << nRet << ']';

        return false;
    }

    bool IsRGBCamera(MvGvspPixelType enType)
    {
        switch (enType)
        {
        case PixelType_Gvsp_BGR8_Packed:
        case PixelType_Gvsp_YUV422_Packed:
        case PixelType_Gvsp_YUV422_YUYV_Packed:
        case PixelType_Gvsp_BayerGR8:
        case PixelType_Gvsp_BayerRG8:
        case PixelType_Gvsp_BayerGB8:
        case PixelType_Gvsp_BayerBG8:
        case PixelType_Gvsp_BayerGB10:
        case PixelType_Gvsp_BayerGB10_Packed:
        case PixelType_Gvsp_BayerBG10:
        case PixelType_Gvsp_BayerBG10_Packed:
        case PixelType_Gvsp_BayerRG10:
        case PixelType_Gvsp_BayerRG10_Packed:
        case PixelType_Gvsp_BayerGR10:
        case PixelType_Gvsp_BayerGR10_Packed:
        case PixelType_Gvsp_BayerGB12:
        case PixelType_Gvsp_BayerGB12_Packed:
        case PixelType_Gvsp_BayerBG12:
        case PixelType_Gvsp_BayerBG12_Packed:
        case PixelType_Gvsp_BayerRG12:
        case PixelType_Gvsp_BayerRG12_Packed:
        case PixelType_Gvsp_BayerGR12:
        case PixelType_Gvsp_BayerGR12_Packed:
            return true;
        default:
            return false;
        }
    }

    void UnloadCamera() {
        int nRet;
        nRet = MV_CC_StopGrabbing(_handle);
        if (MV_OK != nRet)
            LOG(WARNING) << "Failed to stop grabbing. nRet [" << nRet << ']';

        nRet = MV_CC_CloseDevice(_handle);
        if (MV_OK != nRet)
            LOG(WARNING) << "Failed to close device. nRet [" << nRet << ']';

        nRet = MV_CC_DestroyHandle(_handle);
        if (MV_OK != nRet)
            LOG(WARNING) << "Failed to destroy handle. nRet [" << nRet << ']';
    }

public:
    HikCameraCapture() {
        if (!InitCamera(nullptr))
            throw_with_trace(HikCameraCaptureException, "HikCamera init failed, see log for details.");
    }

    HikCameraCapture(const char* userDefinedName) {
        if (!InitCamera(userDefinedName))
            throw_with_trace(HikCameraCaptureException, "HikCamera init failed, see log for details.");
    }

    HikCameraCapture(const HikCameraCapture&) = delete;
    HikCameraCapture(HikCameraCapture&&) = delete;

    ~HikCameraCapture() {
        UnloadCamera();
    }

    std::tuple<cv::Mat, TimeStamp> Read() override {
        MV_FRAME_OUT stImageInfo;
        std::tuple<cv::Mat, TimeStamp> result;
        auto& [img, timeStamp] = result;

        if constexpr (debugCanvas.master) {
            if (debugCanvas.master.DebugFrameHandler.Paused && pConvertData != nullptr) {
                img = cv::Mat(stConvertParam.nHeight, stConvertParam.nWidth, CV_8UC3, stConvertParam.pDstBuffer);
                timeStamp = TimeStampCounter::GetTimeStamp();
                return result;
            }
        }

        int nRet = MV_CC_GetImageBuffer(_handle, &stImageInfo, 1000);
        timeStamp = TimeStampCounter::GetTimeStamp();
        if (nRet == MV_OK)
        {
            // 注意：为了最大化性能，这里只考虑相机传入的每帧图像大小、格式不变的情况，如有新的情况请修改代码
            if (pConvertData == nullptr) {
                if (!IsRGBCamera(stImageInfo.stFrameInfo.enPixelType))
                    throw_with_trace(HikCameraCaptureException, "RGB camera needed!");

                ConvertDataSize = stImageInfo.stFrameInfo.nWidth * stImageInfo.stFrameInfo.nHeight * 3;
                pConvertData = new unsigned char[ConvertDataSize];

                stConvertParam.nWidth = stImageInfo.stFrameInfo.nWidth;                 // image width
                stConvertParam.nHeight = stImageInfo.stFrameInfo.nHeight;               // image height                
                stConvertParam.nSrcDataLen = stImageInfo.stFrameInfo.nFrameLen;         // input data size
                stConvertParam.enSrcPixelType = stImageInfo.stFrameInfo.enPixelType;    // input pixel format
                stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;             // output pixel format
                stConvertParam.pDstBuffer = pConvertData;                               // output data buffer
                stConvertParam.nDstBufferSize = ConvertDataSize;                        // output buffer size
            }

            stConvertParam.pSrcData = stImageInfo.pBufAddr;                             // input data buffer

            nRet = MV_CC_ConvertPixelType(_handle, &stConvertParam);
            if (MV_OK != nRet) {
                LOG(ERROR) << "nRet [" << nRet << ']';
                throw_with_trace(HikCameraCaptureException, "Failed to convert Pixel Type!")
            }

            // 注意：这是cv::Mat的一个特殊构造函数，Mat指向的图像内容不会随Mat的析构被其自动析构，处理不当会造成内存泄露
            img = cv::Mat(stImageInfo.stFrameInfo.nHeight, stImageInfo.stFrameInfo.nWidth, CV_8UC3, stConvertParam.pDstBuffer);

            // 这里不涉及堆内存释放
            MV_CC_FreeImageBuffer(_handle, &stImageInfo);

            return result;
        }
        else {
            LOG(ERROR) << "nRet [" << nRet << ']';
            throw_with_trace(HikCameraCaptureException, "No frame data!")
        }
    }
};

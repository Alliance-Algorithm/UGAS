#pragma once
/*
Creation Date: 2023/05/19
Latest Update: 2023/05/19
Developer(s): 21-WTR
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- read video stream from UDP sources (etc. unity) and return needed img data
- only works on Windows
*/

#include "opencv2/imgcodecs/legacy/constants_c.h"
#include "Core/ImgCapture/ImgCaptureInterface.h"
#include <vector>
#include "Util/Debug/Log.h"
#include "Util/Debug/DebugCanvas.h"
#pragma comment(lib,"ws2_32.lib")

class UDPVideoCapture : public ImgCaptureInterface, private cv::VideoCapture {
private:
	WSADATA WSAData;
	WORD sockVersion = MAKEWORD(2, 2); 
	SOCKET serSocket;

	sockaddr_in clientAddr;
	int iAddrlen = sizeof(clientAddr);
	char buff[65536];

public:
	explicit UDPVideoCapture(const int port) {
		if (WSAStartup(sockVersion, &WSAData) != 0)
			LOG(ERROR) << "Windows socket asynchronous startup error";
		
		serSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
		if (INVALID_SOCKET == serSocket)
			LOG(ERROR) << "Socket error";

		sockaddr_in serAddr;
		serAddr.sin_family = AF_INET;
		serAddr.sin_port = htons(port);
		serAddr.sin_addr.S_un.S_addr = INADDR_ANY;

		if (bind(serSocket, (sockaddr*)&serAddr, 
			sizeof(serAddr)) == SOCKET_ERROR)
		{
			LOG(ERROR) << "Bind error";
			closesocket(serSocket);
		}
	}
	UDPVideoCapture(const UDPVideoCapture&) = delete;
	UDPVideoCapture(UDPVideoCapture&&) = delete;

	std::tuple<cv::Mat, TimeStamp> Read() override {
		std::tuple<cv::Mat, TimeStamp> result;
		auto& [img, timestamp] = result;

		std::vector<unsigned char> decode;
		memset(buff, 0, sizeof(buff));
		int len = recvfrom(serSocket, buff, sizeof(buff), 0, (sockaddr*)&clientAddr, &iAddrlen); 
		//LOG(INFO) << "received " << len << " bytes";
		int pos = 0;
		while (pos < len) {
			decode.push_back(buff[pos++]); 
		}
		img = cv::imdecode(decode, CV_LOAD_IMAGE_COLOR);

		if (img.empty()) {
			throw_with_trace(std::runtime_error, "Read empty frame!");
		}
		timestamp = TimeStampCounter::GetTimeStamp();
		return result;
	}
};

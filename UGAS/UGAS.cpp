#include "UGAS.h"
using namespace std;
using namespace cv;

UGAS::UGAS() :
	_com(*new GIMBAL_SERIAL()),
	_imgCapture(*new IMG_CAPTURE())
{
	_com.Open(SERIAL_PORT);
	_com.RecvGimbalData();
	_imgCapture.init(&video);
	ParametersInit(static_cast<Team>(_com.team));
}

UGAS::~UGAS() {
	delete& _com;
	delete& _imgCapture;
}

void UGAS::initial() {
	try {
		// 初始化部分

	}
	catch (const char* str) {
		cout << str;
		throw;
	}
	catch (...) {
		cout << "Unkown Error!";
		throw;
	}
}

void UGAS::always() {
	while (true) {
		try {
			// 主要工作循环

		}
		catch (const char* str) {
			cout << str;
			throw;
		}
		catch (...) {
			cout << "Unkown Error!";
			throw;
		}
	}
}

#include "UGAS.h"
using namespace std;
using namespace cv;

UGAS::UGAS() :
	_com(*new GIMBAL_SERIAL()),
	_imgCapture(*new IMG_CAPTURE()),
	_pretreater(*new IMG_PRETREAT(_com)),
	_armorIdentifier(*new ARMOR_IDENTIFY(_com, *new NUMBER_IDENTIFY()))
{
	_com.Open(SERIAL_PORT);
	_com.RecvGimbalData();
	_imgCapture.init(&video);
	ParametersInit(static_cast<Team>(_com.team));
}

UGAS::~UGAS() {
	delete& _com;
	delete& _imgCapture;
	delete& _pretreater;
	delete& _armorIdentifier;

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
	// 中间过程变量
	Img					img;
	vector<ArmorPlate>	armors;
	
	while (true) {
		try {
			// 主要工作循环

			START_COUNT
			_imgCapture.read(img);
			PRINT_COST
			END_COUNT

			START_COUNT
			_pretreater.GetPretreated(img);
			PRINT_COST
			END_COUNT

			START_COUNT
			_armorIdentifier.Identify(img, armors);
			PRINT_COST
			END_COUNT

			putchar('\n');

			//_fps.Count();
			//printf("\rNow time stamp:%llu | Fps: %3d     ",
			//	TimeStampCounter::GetTimeStamp(), _fps.GetFPS());
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

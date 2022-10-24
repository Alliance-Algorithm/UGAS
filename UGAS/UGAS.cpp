#include "UGAS.h"
using namespace std;
using namespace cv;

UGAS::UGAS() :
	_com(*new GIMBAL_SERIAL()),
	_imgCapture(*new IMG_CAPTURE()),
	_pretreater(*new IMG_PRETREAT(_com)),
	_armorIdentifier(*new ARMOR_IDENTIFY(_com, *new NUMBER_IDENTIFY())),
	_targetSolution(*new TARGET_SOLUTION(_com)),
	_trackingStrategy(*new TRACK_STRATEGY(_com)),
	_trajectory(*new TRAJECTORY(_com))
	{}

UGAS::~UGAS() {
	delete& _com;
	delete& _imgCapture;
	delete& _pretreater;
	delete& _armorIdentifier;
	delete& _targetSolution;
	delete& _trackingStrategy;
	delete& _trajectory;
}

void UGAS::initial() {
	try {
		// 初始化部分
		_com.Open(SERIAL_PORT);
		_com.RecvGimbalData();
		_imgCapture.init(&video);
		ParametersInit(static_cast<Team>(_com.team));
	}
	catch (const char* str) {
		throw_with_trace(std::runtime_error, str);
	}
	catch (...) {
		throw_with_trace(std::runtime_error, "Unkown Error!");
	}
}

void UGAS::always() {
	// 中间过程变量
	Img						img;
	vector<ArmorPlate>		armors;
	const vector<Target>&	targets = _targetSolution.GetResultRefer();
	Target					aimingTarget;
	double yaw, pitch;

	while (true) {
		try {
			// 主要工作循环
			_imgCapture.read(img);
			_pretreater.GetPretreated(img);
			_armorIdentifier.Identify(img, armors);
			_targetSolution.Solve(armors);
			_trackingStrategy.GetTarget(targets, aimingTarget);
			_trajectory.GetShotAngle(aimingTarget, img.timeStamp, yaw, pitch);
			_com.SetAngle(yaw, pitch - _com.pitchA);
			_com.Send();

			_fps.Count();
			printf("\rNow time stamp:%llu | Fps: %3d     ",
				TimeStampCounter::GetTimeStamp(), _fps.GetFPS());
		}
		catch (const char* str) {
			throw_with_trace(std::runtime_error, str);
		}
		catch (...) {
			throw_with_trace(std::runtime_error, "Unkown Error!");
		}
	}
}

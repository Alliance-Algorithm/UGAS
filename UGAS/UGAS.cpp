#include "UGAS.h"
using namespace std;
using namespace cv;

UGAS::UGAS() :
	_imgCapture(*new IMG_CAPTURE()),
	_pretreater(*new IMG_PRETREAT(com)),
	_armorIdentifier(*new ARMOR_IDENTIFY(*new NUMBER_IDENTIFY())),
	_targetSolution(*new TARGET_SOLUTION()),
	_trackingStrategy(*new TRACK_STRATEGY()),
	_trajectory(*new TRAJECTORY())
	{}

UGAS::~UGAS() {
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
		com.Get().Open(SERIAL_PORT);
		com.Get().RecvGimbalData();
		_imgCapture.init(&video);
		ParametersInit(static_cast<Team>(com.Get().team));
	}
	catch (const char* str) {
		throw_with_trace(std::runtime_error, str);
	}
}

void UGAS::always() {
	// 中间过程变量
	Img					img;
	vector<ArmorPlate>	armors;
	int					targetID;
	double				yaw, pitch;

	while (true) {
		try {
			// 主要工作循环
			com.Get().RecvGimbalData();
			_imgCapture.read(img);
#if		DEBUG_IMG == 1
#if		DEBUG_PRETREAT == 0
			debugImg.Load(img);
			_pretreater.GetPretreated(img);
#else	// DEBUG_PRETREAT == 1
			_pretreater.GetPretreated(img);
			debugImg.Load(img);
#endif	// DEBUG_PRETREAT
#else	// DEBUG_IMG == 0
			_pretreater.GetPretreated(img);
#endif	// DEBUG_IMG
			PnPsolver.GetTransMat();
			_armorIdentifier.Identify(img, armors);
			_targetSolution.Solve(img.timeStamp, armors);
			targetID = _trackingStrategy.GetTargetID();
			if (targetID) {
				_trajectory.GetShotAngle(targetID, img.timeStamp, yaw, pitch);
				com.Get().SetAngle(yaw, pitch);
			}
			else com.Get().SetAngle(.0, .0);
			com.Get().Send();

#if		DEBUG_IMG == 1
			debugImg.Show();
#endif	// DEBUG_IMG

			_fps.Count();
			printf("\rNow time stamp:%llu | Fps: %3d     ",
				TimeStampCounter::GetTimeStamp(), _fps.GetFPS());
		}
		catch (const char* str) {
			throw_with_trace(std::runtime_error, str);
		}
	}
}

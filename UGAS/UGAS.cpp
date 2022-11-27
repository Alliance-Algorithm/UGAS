#include "UGAS.h"
using namespace std;
using namespace cv;

UGAS::UGAS() :
	_imgCapture(*new IMG_CAPTURE()),
	_pretreater(*new IMG_PRETREAT()),
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
		destroyAllWindows();
		com.Get().Open(SERIAL_PORT);
		com.Get().RecvGimbalData();
		_imgCapture.init(&video);
		ParametersInit(static_cast<Team>(com.Get().team));
	}
	catch (const char* str) { // 重包装异常
		throw_with_trace(std::runtime_error, str);
	}
}

void UGAS::always() {
	// 中间过程变量
	Img					img, imgThre, imgGray;
	Rect				ROIregion;
	vector<ArmorPlate>	armors;
	int					targetID;
	double				yaw, pitch;

	// 获取图像大小
	_imgCapture.read(img);
	frameWidth = img.cols; frameHeight = img.rows;
	// 初始化ROI
	ROIregion = Rect(0, 0, frameWidth, frameHeight);

	while (true) {
		try {
			/// =============== 主要工作循环开始 ===============
			com.Get().RecvGimbalData();
			_imgCapture.read(img);

#if		DEBUG_IMG == 1 // 加载调试输出图像
			debugImg.Load(img, ROIregion);
#endif	// DEBUG_IMG

#if ENABLE_ROI == 1 // 取ROI区域
			img.SetROI(ROIregion);
			ROIoffset = ROIregion.tl();
#endif // ENABLE_ROI == 1

			_pretreater.GetPretreated(img, imgThre, imgGray);
			PnPsolver.GetTransMat();
			_armorIdentifier.Identify(imgThre, imgGray, armors);
			_targetSolution.Solve(img.timeStamp, armors);
			targetID = _trackingStrategy.GetTargetID();
			if (targetID) {
				_trajectory.GetShotAngle(targetID, img.timeStamp, yaw, pitch);
				com.Get().SetAngle(yaw, pitch);
			}
			else com.Get().SetAngle(yaw = .0, pitch = .0);
			com.Get().Send();

#if ENABLE_ROI == 1 // 更新ROI
			if (targetID)
				ROIregion = robots[targetID].ROIregion(img.timeStamp);
			else ROIregion = Rect(0, 0, frameWidth, frameHeight);
#endif // ENABLE_ROI == 1

			/// =============== 一些调试信息 ===============
#if		DEBUG_PRETREAT == 1
			imshow("Pretreat", imgThre);
#endif

#if		DEBUG_ANGLE == 1
			MAKE_GRAGH_DEFAULT
				GRAGH_ADD_VAR(yaw, COLOR_YELLOW)
				GRAGH_ADD_VAR(pitch, COLOR_BLUE)
			SHOW_GRAGH(Gragh_Yaw_Pitch)
#endif // DEBUG_ANGLE == 1

#if		DEBUG_IMG == 1 //输出至图像
			_fps.Count();
			_fps.PrintFPS(debugImg);
			debugImg.Show();
#else		//直接打印在控制台上
			printf("\rNow time stamp:%llu | Fps: %3d     ",
				TimeStampCounter::GetTimeStamp(), _fps.Count());
#endif	// DEBUG_IMG == 1
			/// =============== 主要工作循环结束 ===============
		}
		catch (const char* str) { // 重包装异常
			throw_with_trace(std::runtime_error, str);
		}
	}
}

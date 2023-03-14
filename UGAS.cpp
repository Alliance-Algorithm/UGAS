#include "UGAS.h"
using namespace std;
using namespace cv;

UGAS::UGAS() :
	_imgCapture(new ResizeCapture<CVVideoCapture>(0.5, "Blue_4.mp4")),
	_pretreater(new ImgPretreat_V1()),
	_armorIdentifier(new ArmorIdentifier_V1(*new NumberIdentifier_V1())),
	_targetSolution(new TargetSolution_V1()),
	_trackingStrategy(new TrackingStrategy_V1()),
	_trajectory(new Trajectory_FEM()) {
}

UGAS::~UGAS() { }

void UGAS::initial() {
	try {
		// 初始化部分
		destroyAllWindows();
		com.Get().Open(SERIAL_PORT);
		com.Get().RecvGimbalData();
		ParametersInit(static_cast<Team>(com.Get().team));
	}
	catch (const char* str) { // 重包装异常
		throw_with_trace(std::runtime_error, str);
	}
}

void UGAS::always() {
	// 中间过程变量
	Rect				ROIregion;
	int					targetID;
	double				yaw, pitch;

	// 获取图像大小
	auto [img, timeStamp] = _imgCapture->Read();
	//frameWidth = img.cols; frameHeight = img.rows;
	// 初始化ROI
	//ROIregion = Rect(0, 0, frameWidth, frameHeight);

	while (true) {
		try {
			com.Get().RecvGimbalData();
			auto [img, timeStamp] = _imgCapture->Read();

			if constexpr(debugCanvas.master) {
				debugCanvas.master.LoadMat(img);
			}

#if ENABLE_ROI == 1 // 取ROI区域
			img(ROIregion);
			ROIoffset = ROIregion.tl();
#endif // ENABLE_ROI == 1
			auto [imgThre, imgGray] = _pretreater->GetPretreated(img);
			auto armors = _armorIdentifier->Identify(imgThre, imgGray);
			PnPsolver.GetTransMat();

			_targetSolution->Solve(timeStamp, armors);
			targetID = _trackingStrategy->GetTargetID();
			if (targetID) {
				_trajectory->GetShotAngle(targetID, timeStamp, yaw, pitch);
				com.Get().SetAngle(yaw, pitch);
			}
			else com.Get().SetAngle(yaw = .0, pitch = .0);
			com.Get().Send();

#if ENABLE_ROI == 1 // 更新ROI
			if (targetID)
				ROIregion = robots[targetID].ROIregion(img.timeStamp);
			else ROIregion = Rect(0, 0, frameWidth, frameHeight);
#endif // ENABLE_ROI == 1

			if constexpr(debugCanvas.pretreat) {
				debugCanvas.pretreat.LoadMat(imgThre);
			}

#if		DEBUG_ANGLE == 1
			MAKE_GRAGH_DEFAULT
				GRAGH_ADD_VAR(yaw, COLOR_YELLOW)
				GRAGH_ADD_VAR(pitch, COLOR_BLUE)
				SHOW_GRAGH(Gragh_Yaw_Pitch)
#endif // DEBUG_ANGLE == 1

			if constexpr(debugCanvas.fps) {
				_fps.Count();
				_fps.PrintFPS(debugCanvas.fps.GetMat());
			}
			else {
				constexpr int fpsInterval = 100;
				static int i = fpsInterval;
				if (--i == 0) {
					printf("\rNow time stamp:%llu | Fps: %3d       ", TimeStampCounter::GetTimeStamp(), _fps.Count());
					i = fpsInterval;
				}
				else _fps.Count();
			}

			if constexpr(DEBUG_IMG) {
				debugCanvas.ShowAll();
				cv::waitKey(1);
			}
		}
		catch (const char* str) { // 重包装异常
			throw_with_trace(std::runtime_error, str);
		}
	}
}

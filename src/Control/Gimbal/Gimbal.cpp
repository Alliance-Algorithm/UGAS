#include "Gimbal.h"

#include <thread>
#include <opencv2/opencv.hpp>

#include "Core/ImgCapture/Common/CVVideoCapture.h"
#include "Core/ImgCapture/Common/HTCameraCapture.h"
#include "Core/ImgCapture/Common/HikCameraCapture.h"
#include "Core/ImgCapture/Common/ResizeCapture.h"
#include "Core/ImgCapture/Common/RotateCapture.h"
#include "Core/Pretreator/Armor/ArmorPretreator_V1.h"
#include "Core/Pretreator/Armor/ArmorPretreator_V2.h"
#include "Core/Identifier/Armor/ArmorIdentifier_V1.h"
#include "Core/Identifier/Armor/ArmorIdentifier_V2.h"
#include "Core/Identifier/Armor/ArmorIdentifier_V3.h"
#include "Core/Identifier/Number/NullNumberIdentifier.h"
#include "Core/Identifier/Number/NumberIdentifier_V1.h"
#include "Core/Identifier/Color/ColorIdentifier_V1.h"
#include "Core/PnPSolver/Armor/ArmorPnPSolver_V1.h"
#include "Util/Debug/DebugCanvas.h"
#include "Util/Debug/DebugSettings.h"
#include "Util/FPSCounter/FPSCounter.h"
#include "Util/Debug/MatForm/RectangleControl.hpp"


void Gimbal::Always() const {
	auto imgCapture = RotateCapture<HikCameraCapture>(cv::RotateFlags::ROTATE_180);

	//auto armorPretreator = ArmorPretreator_V2();
	auto armorIdentifier = ArmorIdentifier_V3<NumberIdentifier_V1>("models/NumberIdentifyModelV0.pb");
	//auto armorIdentifier = ArmorIdentifier_V3<NullNumberIdentifier>();
	auto pnpSolver = ArmorPnPSolver_V1();

	auto _fps = FPSCounter();

	/*auto panel = std::unique_ptr<RectangleControl>();
	cv::Mat imghsv;
	if constexpr (debugCanvas.master) {
		panel = std::make_unique<RectangleControl>();
		auto& matform = debugCanvas.master.GetMatForm();
		panel->OnMouseDown = [&imghsv](auto arg) {
			std::cout << imghsv.at<cv::Vec3b>(arg.y, arg.x) << std::endl;
			std::cout << debugCanvas.master.GetMat().at<cv::Vec3b>(arg.y, arg.x) << std::endl;
		};
		panel->ForeColor = COLOR_YELLOW;
		matform.AddControl(panel.get());
	}*/

	while (true) {
		try {
			auto [img, timeStamp] = imgCapture.Read();
			//cv::Mat img2;
			//cv::undistort(img, img2, CameraMatrix, DistCoeffs);
			//imgCapture.Test();

			if constexpr (debugCanvas.master) {
				debugCanvas.master.LoadMat(img);
				//panel->BoundRect = cv::Rect(0, 0, img.cols, img.rows);
				//cv::cvtColor(img, imghsv, cv::COLOR_BGR2HSV_FULL);
			}

			auto armors = armorIdentifier.Identify(img);

			//auto [imgThre, imgGray] = armorPretreator.GetPretreated(img);
			//auto armors = armorIdentifier.Identify(img);

			if (!armors.empty()) {
				pnpSolver.Solve(armors[0], false);
				//std::cout << pnpSolver.Solve(armors[0], false) << std::endl;
			}

			/*if constexpr (debugCanvas.pretreat) {
				debugCanvas.pretreat.LoadMat(imgThre);
			}*/

			if constexpr (debugCanvas.fps) {
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

			if constexpr (DEBUG_IMG) {
				debugCanvas.ShowAll();
				cv::waitKey(1);
			}
		}
		catch (const char* str) { // 重包装异常
			throw_with_trace(std::runtime_error, str);
		}
	}
}
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
#include "Core/Predictor/Armor/ArmorPredictor_V1.h"
#include "Core/Strategy/Common/Strategy_V1.h"
#include "Core/Trajectory/Common/Trajectory_V1.h"
#include "Control/Serial/SerialCBoard.h"
#include "Util/Debug/DebugCanvas.h"
#include "Util/Debug/DebugSettings.h"
#include "Util/FPSCounter/FPSCounter.h"
#include "Util/Debug/MatForm/RectangleControl.hpp"

void Gimbal::Always() const {
	auto imgCapture = RotateCapture<HikCameraCapture>(cv::RotateFlags::ROTATE_180);

	auto armorIdentifier = ArmorIdentifier_V3<NumberIdentifier_V1>("models/NumberIdentifyModelV0.pb");
	auto pnpSolver = ArmorPnPSolver_V1();
	auto armorPredictor = ArmorPredictor_V1();

	auto strategy = Strategy_V1();
	auto trajectory = Trajectory_V1();

	auto fps = FPSCounter();
	auto cboard = SerialCBoard("COM3");

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
			auto armors3d = std::vector<ArmorPlate3d>();

			for (const auto& armor : armors)
				if (auto&& armor3d = pnpSolver.Solve(armor, false))
					armors3d.push_back(*armor3d);

			const auto& targets = armorPredictor.Update(armors3d, timeStamp);
			
			int targetIndex = strategy.GetTargetIndex(targets);
			if (targetIndex >= 0) {
				auto attitude = trajectory.GetShotAngle(targets[targetIndex]);
				cboard.Send(attitude);
			}

			if constexpr (debugCanvas.fps) {
				fps.Count();
				fps.PrintFPS(debugCanvas.fps.GetMat());
			}
			else {
				constexpr int fpsInterval = 100;
				static int i = fpsInterval;
				if (--i == 0) {
					printf("\rNow time stamp:%llu | Fps: %3d       ", TimeStampCounter::GetTimeStamp(), fps.Count());
					i = fpsInterval;
				}
				else fps.Count();
			}

			if constexpr (DEBUG_IMG) {
				debugCanvas.ShowAll();
				cv::waitKey(1);
			}
		}
		catch (const char* str) { // �ذ�װ�쳣
			throw_with_trace(std::runtime_error, str);
		}
	}
}
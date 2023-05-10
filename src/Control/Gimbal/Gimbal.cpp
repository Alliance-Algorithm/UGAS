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
#include "Core/PnPSolver/Armor/ArmorPnPSolver_V2.h"
#include "Core/Predictor/Armor/ArmorPredictor_V1.h"
#include "Core/Strategy/Common/Strategy_V1.h"
#include "Core/Trajectory/Common/Trajectory_V1.h"
#include "Control/Serial/CBoardInfantry.h"
#include "Control/Serial/GYH1.h"
#include "Control/Serial/HiPNUC.h"
#include "Util/Debug/DebugCanvas.h"
#include "Util/Debug/DebugSettings.h"
#include "Util/FPSCounter/FPSCounter.h"
#include "Util/Debug/MatForm/RectangleControl.hpp"

void Gimbal::Always() const {
	auto imgCapture = RotateCapture<HikCameraCapture>(cv::RotateFlags::ROTATE_180);
	// auto imgCapture = CVVideoCapture("Blue_4.mp4");

	auto hipnuc = HiPNUC("COM14");
	auto cboard = CBoardInfantry("COM16");

	auto armorIdentifier = ArmorIdentifier_V3<NumberIdentifier_V1>("models/NumberIdentifyModelV3.pb");

	auto pnpSolver = ArmorPnPSolver_V1();
	// auto pnpSolver = ArmorPnPSolver_V2<HiPNUC>(hipnuc);
	auto armorPredictor = ArmorPredictor_V1();
	auto strategy = Strategy_V1();
	auto trajectory = Trajectory_V1();

	auto fps = FPSCounter_V2();

	GimbalAttitude attitude;

	while (true) {
		try {

			cboard.Receive();
			auto [img, timeStamp] = imgCapture.Read();

			if constexpr (debugCanvas.master) {
				debugCanvas.master.LoadMat(img);
			}

			auto armors = armorIdentifier.Identify(img, cboard.GetEnemyColor());
			auto armors3d = std::vector<ArmorPlate3d>();

			for (const auto& armor : armors)
				if (auto&& armor3d = pnpSolver.Solve(armor))
					armors3d.push_back(*armor3d);

			const auto& targets = armorPredictor.Update(armors3d, timeStamp);
			
			int targetIndex = strategy.GetTargetIndex(targets);
			if (targetIndex >= 0) {
				attitude = trajectory.GetShotAngle(targets[targetIndex]);
				cboard.Send(attitude);
			}
			else {
				cboard.Send({0, 0});
			}

			if constexpr (DEBUG_IMG) {
				debugCanvas.ShowAll();
				cv::waitKey(1);
			}

			if (fps.Count()) {
				std::cout << "Fps: " << fps.GetFPS() << '\n';
			}

		}
		catch (const char* str) { // 重包装异常
			throw_with_trace(std::runtime_error, str);
		}
	}

}
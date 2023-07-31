#include "Gimbal.h"

#include <thread>
#include <opencv2/opencv.hpp>

#include "config.h"
#include "Core/ImgCapture/Common/CVVideoCapture.h"
#include "Core/ImgCapture/Common/HTCameraCapture.h"
#include "Core/ImgCapture/Common/HikCameraCapture.h"
#include "Core/ImgCapture/Common/ResizeCapture.h"
#include "Core/ImgCapture/Common/RotateCapture.h"
#include "Core/ImgCapture/Common/ImageFolderCapture.h"
#include "Core/Pretreator/Armor/ArmorPretreator_V1.h"
#include "Core/Pretreator/Armor/ArmorPretreator_V2.h"
#include "Core/Identifier/Armor/ArmorIdentifier_V1.h"
#include "Core/Identifier/Armor/ArmorIdentifier_V2.h"
#include "Core/Identifier/Armor/ArmorIdentifier_V3.h"
#include "Core/Identifier/Number/NullNumberIdentifier.h"
#include "Core/Identifier/Number/NumberIdentifier_V1.h"
#include "Core/Identifier/Number/NumberIdentifier_V2.h"
#include "Core/Identifier/Color/ColorIdentifier_V1.h"
#include "Core/PnPSolver/Armor/ArmorPnPSolver.h"
#include "Core/Predictor/Armor/SimplePredictor.h"
#include "Core/Tracker/Armor/ArmorEKFTracker_V3.h"
#include "Core/Strategy/Common/Strategy_V1.h"
#include "Core/Trajectory/Common/Trajectory_V1.h"
#include "Control/Serial/CBoardInfantry.h"
#include "Control/Serial/CBoardSentry.h"
#include "Control/Serial/VirtualCBoard.h"
#include "Control/Serial/GYH1.h"
#include "Control/Serial/HiPNUC.h"
#include "Util/Debug/DebugCanvas.h"
#include "Util/Recorder/PNGRecorder.h"
#include "Util/FPSCounter/FPSCounter.h"
#include "Util/Debug/MatForm/RectangleControl.hpp"

[[noreturn]] void Gimbal::Always() {
    //auto imgCapture = HikCameraCapture();
    auto imgCapture = RotateCapture<HikCameraCapture>(cv::RotateFlags::ROTATE_180);
    //auto imgCapture = ImageFolderCapture("../UGAS-record/sentry-record-230610-8am/");
    //auto imgCapture = CVVideoCapture("Blue_4.mp4");

    auto hipnuc = HiPNUC("/dev/IMU");

    auto cboard = CBoardInfantry("/dev/CBoard");
    //auto cboard = CBoardInfantry("/dev/ttyUSB1");
    //auto cboard = VirtualCBoard(true);

    //auto armorIdentifier = ArmorIdentifier_V3<NumberIdentifier_V2>("../models/NumberIdentifyModelV3.pb", "../models/mlp.onnx");
    auto armorIdentifier = ArmorIdentifier_V3<NumberIdentifier_V1>("models/NumberIdentifyModelV4.pb");

    auto simplePredictor = SimplePredictor();
    auto ekfTracker = ArmorEKFTracker();
    auto strategy = Strategy_V1();
    auto trajectory = Trajectory_V1();

    auto fps = FPSCounter_V2();

    auto recorder = PNGRecorder("images/", ENABLE_RECORDING ? 3.0 : 0.0);

    bool autoscopeEnabled = false;

    double angle = 0;
    int interval = 0;
    while (true) {
        cboard.Receive();

        auto [img, _discard_] = imgCapture.Read();
        auto timestamp = std::chrono::steady_clock::now();
        bool imuAvailable = hipnuc.UpdateTransformer();

        if constexpr (debugCanvas.master) {
            debugCanvas.master.LoadMat(img);
        }

        auto armors = armorIdentifier.Identify(img, cboard.GetEnemyColor());

        if (imuAvailable) {
            // 陀螺仪工作正常
            auto armors3d = ArmorPnPSolver::SolveAll(armors);

            autoscopeEnabled = cboard.GetAutoscopeEnabled();

            if (auto target = ekfTracker.Update(armors3d, timestamp)) {
                auto [yaw, pitch] = trajectory.GetShotAngle(*target, cboard.GetBulletSpeed());
                // sentry y+1.7 p+0.0
                yaw += 0.4 / 180.0 * MathConsts::Pi;
                pitch += 2.4 / 180.0 * MathConsts::Pi;
                cboard.Send(yaw, pitch);
                recorder.Record(img, timestamp);
            }
            else {
                cboard.Send(0, 0);
            }
        }
        else {
            // 陀螺仪工作不正常，采用低保运行模式
            auto armors3d = ArmorPnPSolver::SolveAll(armors);
            if (auto target = simplePredictor.Update(armors3d, timestamp)) {
                auto [yaw, pitch] = trajectory.GetShotAngle(*target, cboard.GetBulletSpeed());
                yaw += 0.5 / 180.0 * MathConsts::Pi;
                pitch += 2.4 / 180.0 * MathConsts::Pi;
                cboard.Send(yaw, pitch);
                recorder.Record(img, timestamp);
            }
            else {
                cboard.Send(0, 0);
            }
        }

        if constexpr (ENABLE_DEBUG_CANVAS) {
            if (interval-- == 0) {
                debugCanvas.ShowAll();
                cv::waitKey(1);
                interval = 20;
            }
        }

        if (fps.Count()) {
            std::cout << "Fps: " << fps.GetFPS() << '\n';
        }
    }
}

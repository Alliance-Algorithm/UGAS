#include "Gimbal.h"

#include <thread>
#include <opencv2/opencv.hpp>

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
#include "Core/Tracker/Armor/ArmorEKFTracker.h"
#include "Core/Tracker/Armor/VerticalTracker.h"
#include "Core/Strategy/Common/Strategy_V1.h"
#include "Core/Trajectory/Common/Trajectory_V1.h"
#include "Core/Transformer/SimpleTransformer.h"
#include "Control/Serial/CBoardInfantry.h"
#include "Control/Serial/CBoardSentry.h"
#include "Control/Serial/VirtualCBoard.h"
#include "Control/Serial/GYH1.h"
#include "Control/Serial/HiPNUC.h"
#include "Util/Debug/DebugCanvas.h"
#include "Util/Debug/DebugSettings.h"
#include "Util/Recorder/PNGRecorder.h"
#include "Util/FPSCounter/FPSCounter.h"
#include "Util/Debug/MatForm/RectangleControl.hpp"

void Gimbal::Always() {
    //auto imgCapture = HikCameraCapture();
    auto imgCapture = ImageFolderCapture("../UGAS-record/sentry-record-230610-8am/");
    //auto imgCapture = RotateCapture<HikCameraCapture>(cv::RotateFlags::ROTATE_180);
    //auto imgCapture = CVVideoCapture("Blue_4.mp4");

    auto hipnuc = HiPNUC("COM29");

    //auto cboard = CBoardSentry("COM25");
    auto cboard = CBoardInfantry("COM25");
    //auto cboard = VirtualCBoard();

    auto armorIdentifier = ArmorIdentifier_V3<NumberIdentifier_V2>("models/NumberIdentifyModelV3.pb", "models/mlp.onnx");
    //auto armorIdentifier = ArmorIdentifier_V3<NumberIdentifier_V1>("models/NumberIdentifyModelV4.pb");

    auto pnpSolver = ArmorPnPSolver();
    auto simplePredictor = SimplePredictor();
    auto ekfTracker = ArmorEKFTracker();
    auto strategy = Strategy_V1();
    auto trajectory = Trajectory_V1();

    auto fps = FPSCounter_V2();

    auto recorder = PNGRecorder("images/", 0.0);

    bool autoscopeEnabled = false;

    while (true) {
        cboard.Receive();

        auto [img, __discard__] = imgCapture.Read();
        auto timestamp = std::chrono::steady_clock::now();
        auto transformer = hipnuc.GetTransformer();

        if constexpr (debugCanvas.master) {
            debugCanvas.master.LoadMat(img);
        }

        auto armors = armorIdentifier.Identify(img, cboard.GetEnemyColor());

        if (transformer.Available()) {
            // 陀螺仪工作正常
            auto armors3d = pnpSolver.SolveAll(armors, transformer);

            // 自瞄开启瞬间，重置跟踪目标
            if (!autoscopeEnabled && cboard.GetAutoscopeEnabled())
                ekfTracker.lastTarget = ArmorID::Unknown;

            if (auto&&target = ekfTracker.Update(armors3d, timestamp, transformer)) {
                //auto&& pos = target->Predict(0);
                //std::cout << pos.x() << ' ' << pos.y() << ' ' << pos.z() << '\n';
                auto [yaw, pitch] = trajectory.GetShotAngle(*target, cboard.GetBulletSpeed(), transformer);
                // sentry y+1.7 p+0.0
                yaw += 1.7 / 180.0 * MathConsts::Pi;
                pitch += 0.3 / 180.0 * MathConsts::Pi;
                cboard.Send(yaw, pitch);// , target->Shotable(trajectory._flyTime + 0.1));
                recorder.Record(img, timestamp);

                //std::cout << yaw * 53 << ' ' << pitch * 53 << '\n';
            }
            else {
                cboard.Send(0, 0);//, false);
                //std::cout << "sending zero!\n";
            }
        }
        else {
            // 陀螺仪工作不正常，采用低保运行模式
            auto armors3d = pnpSolver.SolveAll(armors);
            if (auto target = simplePredictor.Update(armors3d, timestamp)) {
                auto [yaw, pitch] = trajectory.GetShotAngle(*target, cboard.GetBulletSpeed());
                yaw += 0.0 / 180.0 * MathConsts::Pi;
                pitch += 1.7 / 180.0 * MathConsts::Pi;
                //cboard.Send(yaw, pitch);// , false);
                cboard.Send(0, 0);
                recorder.Record(img, timestamp);
            }
            else {
                cboard.Send(0, 0);// , false);
            }
        }

        if constexpr (DEBUG_IMG) {
            debugCanvas.ShowAll();
            cv::waitKey(1);
        }
        //cv::imwrite("test.png", img, compression_params);

        if (fps.Count()) {
            std::cout << "Fps: " << fps.GetFPS() << '\n';
        }
    }
}

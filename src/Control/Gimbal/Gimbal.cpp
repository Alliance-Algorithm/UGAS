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
// #include "Core/Identifier/Buff/BuffIdentifier_V3.h"
// #include "Core/Identifier/Buff/BuffIdentifier_V4.h"
// #include "Core/Identifier/Buff/BuffIdentifier_V5.h"
// #include "Core/Identifier/Buff/BuffIdentifier_V6.h"
// #include "Core/Identifier/Buff/BuffIdentifier_V7.h"
#include "Core/PnPSolver/Armor/ArmorPnPSolver.h"
// #include "Core/PnPSolver/Buff/BuffPnPSolver_V1.h"
#include "Core/Predictor/Armor/SimplePredictor.h"
// #include "Core/Predictor/Buff/BuffPredictor_V1.h"
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
#include "Util/Recorder/PNGRecorder.h"
#include "Util/FPSCounter/FPSCounter.h"
#include "Util/Debug/MatForm/RectangleControl.hpp"

[[noreturn]] void Gimbal::Always() {
    auto imgCapture = HikCameraCapture();
    //auto imgCapture = RotateCapture<HikCameraCapture>(cv::RotateFlags::ROTATE_180);
    //auto imgCapture = ImageFolderCapture("../UGAS-record/sentry-record-230610-8am/");
    //auto imgCapture = CVVideoCapture("Blue_4.mp4");

    auto hipnuc = HiPNUC("COM29");

    //auto cboard = CBoardSentry("COM25");
    auto cboard = CBoardInfantry("COM25");
    //auto cboard = VirtualCBoard();

    /********************************/

    auto armorIdentifier = ArmorIdentifier_V3<NumberIdentifier_V2>("../models/NumberIdentifyModelV3.pb", "../models/mlp.onnx");
    //auto armorIdentifier = ArmorIdentifier_V3<NumberIdentifier_V1>("models/NumberIdentifyModelV4.pb");

    auto pnpSolver = ArmorPnPSolver();
    auto simplePredictor = SimplePredictor();
    auto ekfTracker = ArmorEKFTracker();
    auto strategy = Strategy_V1();

    /********************************/

    // auto buffIdentifier = BuffIdentifier_V7();
    // auto buffPredictor = BuffPredictor_V1();

    /********************************/

    auto trajectory = Trajectory_V1();

    //auto fpsV1 = FPSCounter_V1();
    auto fps = FPSCounter_V2();
    //auto fps_buff = FPSCounter_V2();

    auto recorder = PNGRecorder("../images/", ENABLE_RECORDING ? 3.0 : 0.0);

    AutoscopeState lastAutoscopeState = AutoscopeState::Disable;
    TimeStamp aimTimeStamp, buffTimeStamp;

    while (true) {
        cboard.Receive();

        auto [img, timestamp_ull] = imgCapture.Read();
        auto timestamp = std::chrono::steady_clock::now();
        auto transformer = hipnuc.GetTransformer();

        if constexpr (debugCanvas.master) {
            debugCanvas.master.LoadMat(img);
        }

        if (false && cboard.GetAutoscopeState() == AutoscopeState::Buff) {

            // buffTimeStamp = timestamp_ull;

			// auto enemyColor = false ? cboard.GetEnemyColor() : ArmorColor::Blue;
			// auto remainingTime = true ? cboard.GetRemainingTime() : 100; // remaining time >200: smallbuff, <200 bigbuff
            // //std::cout << "EnemyColor = " << enemyColor << std::endl;
            // //std::cout << "RemaningTime = " << remaningTime << std::endl;

            // if(transformer.Available()) {
            // // using external gyroscope data while external gyroscope is working normally
            //     //std::cout<<"transformer available"<<std::endl;

            //     //if (auto&& buffIdentifyData = buffIdentifier.Identify(img, timestamp_ull, enemyColor)) { // V3/V4/V5
            //     //if (auto&& buffIdentifyData = buffIdentifier.Update(img, timestamp_ull, enemyColor)) { // V6
            //     if (auto&& buffIdentifyData = buffIdentifier.Update(img, timestamp_ull)) { //V7
            //         buffPredictor.Update(*buffIdentifyData, transformer, remainingTime);
            //     //     std::cout<<"update target"<<std::endl;
            //     //    if (fps_buff.Count()) {
            //     //        std::cout << "buff fps: " << fps_buff.GetFPS() << '\n';
            //     //    }
            //     }
            //     if (auto&& attitude = trajectory.GetShotAngle(buffPredictor, transformer)) {
            //         //std::cout<<"trajectory angle"<<std::endl;
            //         if (timestamp_ull - aimTimeStamp > 100) {
            //             //cboard.Send((*attitude).yaw * MathConsts::Pi / 180.0, (*attitude).pitch * MathConsts::Pi / 180.0);
            //             //cboard.Send((*attitude).yaw, (*attitude).pitch);
            //         }
            //     }
            //     else {
            //         //std::cout<<"no trajectory angle"<<std::endl;
            //         //cboard.Send(0.0, 0.0); // using the world coordinate, buff needn't send (0,0)
            //     }
            // }

            // else {
            // // using cboard gyroscope data while external gyroscope is abnormal
            //     std::cout<<"transformer unavailable"<<std::endl;

            // //    if (auto&& buffIdentifyData = buffIdentifier.Identify(img, timestamp_ull, enemyColor)) {
            // //    //if (auto&& buffIdentifyData = buffIdentifier.Update(img, timestamp_ull)) {
            // //        buffPredictor.Update(*buffIdentifyData, cboard.GetGyrosAttitude(), remainingTime);
            // //        //std::cout<<"update target"<<std::endl;

            // //        if (auto&& attitude = trajectory.GetShotAngle(buffPredictor, cboard.GetGyrosAttitude())) {
            // //            //std::cout<<"trajectory angle"<<std::endl;

            // //            //std::cout << "buff has run for " << timeStamp - aimTimeStamp << "ms" << std::endl;
            // //            if (timestamp_ull - aimTimeStamp > 100) { // jump over the first several attitude result (in 100ms)
            // //                cboard.Send((*attitude).yaw * MathConsts::Pi / 180.0, (*attitude).pitch * MathConsts::Pi / 180.0);
            // //            }
            // //        }
            // //        else {
            // //            //std::cout<<"no trajectory angle"<<std::endl;
            // //            //cboard.Send(0.0, 0.0); // using the world coordinate, buff needn't send (0,0)
            // //        }
            // //    }
            // //    else {
            // //        //cboard.Send(0.0, 0.0);
            // //        //std::cout<<"no target"<<std::endl;
            // //    }
            // }

        }

        else {

            aimTimeStamp = timestamp_ull;

            auto armors = armorIdentifier.Identify(img, cboard.GetEnemyColor());

            if (transformer.Available()) {
	            // 陀螺仪工作正常
	            auto armors3d = pnpSolver.SolveAll(armors, transformer);

	            // 自瞄开启瞬间，重置跟踪目标
	            if (lastAutoscopeState == AutoscopeState::Disable &&
                    cboard.GetAutoscopeState() != AutoscopeState::Disable)
	                ekfTracker.lastTarget = ArmorID::Unknown;
                lastAutoscopeState = cboard.GetAutoscopeState();

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

        }

        //if (fpsV1.Count()) {
        //    fpsV1.PrintFPS(debugCanvas.fps.GetMat());
        //}
        if (fps.Count()) {
            std::cout << "gimbal fps: " << fps.GetFPS() << '\n';
        }

        if constexpr (ENABLE_DEBUG_CANVAS) {
            debugCanvas.ShowAll();
            cv::waitKey(1);
        }

        //cv::imwrite("test.png", img, compression_params);
    }
}

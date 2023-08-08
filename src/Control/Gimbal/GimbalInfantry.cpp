#include "GimbalInfantry.h"

#include <thread>
#include <opencv2/opencv.hpp>

#include "config.h"
#include "Core/ImgCapture/Common/CVVideoCapture.h"
#include "Core/ImgCapture/Common/HikCameraCapture.h"
#include "Core/ImgCapture/Common/ImageFolderCapture.h"
#include "Core/Pretreator/Armor/ArmorPretreator_V2.h"
#include "Core/Identifier/Armor/ArmorIdentifier_V3.h"
#include "Core/Identifier/Buff/BuffIdentifier_V1.h"
#include "Core/Identifier/Number/NumberIdentifier_V1.h"
#include "Core/PnPSolver/Armor/ArmorPnPSolver.h"
#include "Core/PnPSolver/Buff/BuffPnPSolver.h"
#include "Core/Predictor/Armor/SimplePredictor.h"
#include "Core/Tracker/Armor/ArmorEKFTracker_V3.h"
#include "Core/Tracker/Buff/BuffTracker.h"
#include "Core/Trajectory/Common/Trajectory_V1.h"
#include "Control/Serial/CBoardInfantry.h"
#include "Control/Serial/VirtualCBoard.h"
#include "Util/Debug/DebugCanvas.h"
#include "Util/Recorder/PNGRecorder.h"
#include "Util/FPSCounter/FPSCounter.h"
#include "Util/ROS/TfBroadcast.h"

[[noreturn]] void GimbalInfantry::Always() {
    auto img_capture = HikCameraCapture();
//    auto img_capture = CVVideoCapture("./videos/test_vid.mp4");

    auto imu = HiPNUC("/dev/IMU");

    auto cboard = CBoardInfantry("/dev/CBoard");
//    auto cboard = VirtualCBoard();

    auto armor_identifier = ArmorIdentifier_V3<NumberIdentifier_V1>("models/NumberIdentifyModelV4.pb");
    auto buff_identifier = BuffIdentifier_V1("models/buff_nocolor_v6.onnx");

    auto simple_predictor = SimplePredictor();
    auto ekf_tracker = ArmorEKFTracker();
    auto buff_tracker = BuffTracker();
    auto trajectory = Trajectory_V1();

    auto fps = FPSCounter_V2();

    auto recorder = PNGRecorder("images/", ENABLE_RECORDING ? 3.0 : 0.0);

    bool autoscope_enabled, buff_enabled;

    while (true) {
        cboard.Receive();

        auto [img, _discard_] = img_capture.Read();
        auto timestamp = std::chrono::steady_clock::now();
        bool imuAvailable = imu.UpdateTransformer();

        if constexpr (debugCanvas.master) {
            debugCanvas.master.LoadMat(img);
        }

        // autoscope_enabled = cboard.get_auto_scope_enabled();

        if (imuAvailable) {
            if (!buff_enabled && cboard.get_buff_mode_enabled())
                buff_tracker.ResetAll();
            buff_enabled = cboard.get_buff_mode_enabled();

            if (!buff_enabled) {
                auto armors = armor_identifier.Identify(img, cboard.get_enemy_color());
                auto armors3d = ArmorPnPSolver::SolveAll(armors);

                if (auto target = ekf_tracker.Update(armors3d, timestamp)) {
                    auto [yaw, pitch] = trajectory.GetShotAngle(*target, cboard.get_bullet_speed());
                    auto [rect_x, rect_y] = ArmorPnPSolver::ReProjection(target->Predict(0));
                    yaw += parameters::StaticYawOffset;
                    pitch += parameters::StaticPitchOffset;
                    cboard.Send(yaw, pitch, rect_x, rect_y);
                    recorder.Record(img, timestamp);
                }
                else cboard.Send();
            }
            else {
                if (auto buff = buff_identifier.Identify(img)) {
                    if (auto buff3d = BuffPnPSolver::Solve(*buff)) {
                        if (auto target = buff_tracker.Update(*buff3d, timestamp)) {
                            auto [yaw, pitch] = trajectory.GetShotAngle(*target, cboard.get_bullet_speed());
                            auto [rect_x, rect_y] = ArmorPnPSolver::ReProjection(target->Predict(0));
                            yaw += parameters::StaticYawOffset;
                            pitch += parameters::StaticPitchOffset;
                            cboard.Send(yaw, pitch, rect_x, rect_y);
                        }
                    }
                }
                else cboard.Send();
            }

            // 陀螺仪工作正常
        }
        else {
            // 忘了低保那点东西吧
        }


        if constexpr (ENABLE_DEBUG_CANVAS) {
            debugCanvas.ShowAll();
            cv::waitKey(1);
            static int interval = 0;
            if (interval-- == 0) {
                interval = 20;
            }
        }

        if (fps.Count()) {
            std::cout << "Fps: " << fps.GetFPS() << '\n';
        }
    }
}

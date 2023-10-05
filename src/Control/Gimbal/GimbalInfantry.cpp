#include "GimbalInfantry.h"

#include <thread>
#include <opencv2/opencv.hpp>

#include "config.h"
#include "Core/ImgCapture/Common/CVVideoCapture.h"
#include "Core/ImgCapture/Common/HikCameraCapture.h"
#include "Core/ImgCapture/Common/ImageFolderCapture.h"
#include "Core/Pretreator/Armor/ArmorPretreator_V2.h"
#include "Core/Identifier/Armor/ArmorIdentifier_V3.h"
#include "Core/Identifier/Number/NumberIdentifier_V1.h"
#include "Core/PnPSolver/Armor/ArmorPnPSolver.h"
#include "Core/Predictor/Armor/SimplePredictor.h"
#include "Core/Tracker/Armor/ArmorEKFTracker_V3.h"
#include "Core/Trajectory/Common/Trajectory_V1.h"
#include "Control/Serial/CBoardInfantry.h"
#include "Control/Serial/VirtualCBoard.h"
#include "Util/Debug/DebugCanvas.h"
#include "Util/Recorder/PNGRecorder.h"
#include "Util/FPSCounter/FPSCounter.h"

[[noreturn]] void GimbalInfantry::Always() {
    auto imgCapture = HikCameraCapture();

    auto hipnuc = HiPNUC("/dev/IMU");

    auto cboard = CBoardInfantry("/dev/CBoard");
//    auto cboard = VirtualCBoard();

    auto armorIdentifier = ArmorIdentifier_V3<NumberIdentifier_V1>("models/NumberIdentifyModelV4.pb");

    auto simplePredictor = SimplePredictor();
    auto ekfTracker = ArmorEKFTracker();
    auto trajectory = Trajectory_V1();

    auto fps = FPSCounter_V2();

    auto recorder = PNGRecorder("images/", ENABLE_RECORDING ? 3.0 : 0.0);

    bool autoscopeEnabled = false;

    while (true) {
        cboard.Receive();

        auto [img, _discard_] = imgCapture.Read();
        auto timestamp = std::chrono::steady_clock::now();
        bool imuAvailable = hipnuc.UpdateTransformer();

        if constexpr (debugCanvas.master) {
            debugCanvas.master.LoadMat(img);
        }

        auto armors = armorIdentifier.Identify(img, cboard.get_enemy_color());

        if (imuAvailable) {
            // 陀螺仪工作正常
            auto armors3d = ArmorPnPSolver::SolveAll(armors);

            autoscopeEnabled = cboard.get_auto_scope_enabled();

            if (auto target = ekfTracker.Update(armors3d, timestamp)) {
                auto [yaw, pitch] = trajectory.GetShotAngle(*target, cboard.get_bullet_speed());
                auto [rect_x, rect_y] = ArmorPnPSolver::ReProjection(target->Predict(0));
                yaw += parameters::StaticYawOffset;
                pitch += parameters::StaticPitchOffset;
                cboard.Send(yaw, pitch, rect_x, rect_y);
                recorder.Record(img, timestamp);
            }
            else {
                cboard.Send();
            }
        }
        else {
            // 忘了低保那点东西吧
        }

        if constexpr (ENABLE_DEBUG_CANVAS) {
            static int interval = 0;
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

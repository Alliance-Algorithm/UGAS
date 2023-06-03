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
#include "Core/PnPSolver/Armor/ArmorPnPSolver.h"
#include "Core/Predictor/Armor/SimplePredictor.h"
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
#include "Util/FPSCounter/FPSCounter.h"
#include "Util/Debug/MatForm/RectangleControl.hpp"

void Gimbal::Always() {
    auto imgCapture = HikCameraCapture();
    //auto imgCapture = CVVideoCapture("Blue_4.mp4");

    auto hipnuc = HiPNUC("COM23");
    auto cboard = CBoardSentry("COM28");
    //auto cboard = VirtualCBoard();

    auto armorIdentifier = ArmorIdentifier_V3<NumberIdentifier_V1>("models/NumberIdentifyModelV3.pb");

    //auto pnpSolver = ArmorPnPSolver_V1();
    auto pnpSolver = ArmorPnPSolver();
    auto armorPredictor = SimplePredictor();
    auto strategy = Strategy_V1();
    auto trajectory = Trajectory_V1();

    auto fps = FPSCounter_V2();

    while (true) {
        try {

            //cboard.Receive();
            auto [img, timeStamp] = imgCapture.Read();
            auto transformer = hipnuc.GetTransformer();

            if constexpr (debugCanvas.master) {
                debugCanvas.master.LoadMat(img);
            }

            auto armors = armorIdentifier.Identify(img, cboard.GetEnemyColor());

            if (transformer.Available()) {
                // 陀螺仪工作正常
                auto armors3d = pnpSolver.SolveAll(armors, transformer);
                if (auto target = armorPredictor.Update(armors3d, std::chrono::steady_clock::now())) {
                    auto [yaw, pitch] = trajectory.GetShotAngle(*target, cboard.GetBulletSpeed(), transformer);
                    yaw += -0.5 / 180.0 * MathConsts::Pi;;
                    pitch += -0.4 / 180.0 * MathConsts::Pi;
                    cboard.Send(yaw, pitch);
                    std::cout << yaw * 53 << ' ' << pitch * 53 << '\n';
                }
                else {
                    cboard.Send(0, 0);
                }
            }
            else {
                // 陀螺仪工作不正常，采用低保运行模式
                auto armors3d = pnpSolver.SolveAll(armors);
                if (auto target = armorPredictor.Update(armors3d, std::chrono::steady_clock::now())) {
                    auto [yaw, pitch] = trajectory.GetShotAngle(*target, cboard.GetBulletSpeed());
                    yaw += 0.0 / 180.0 * MathConsts::Pi;
                    pitch += 1.7 / 180.0 * MathConsts::Pi;
                    cboard.Send(yaw, pitch);
                }
                else {
                    cboard.Send(0, 0);
                }
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

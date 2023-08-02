#include <chrono>
#include <thread>

#include <eigen3/Eigen/Core>

#include "Control/Gimbal/GimbalInfantry.h"
#include "Control/Gimbal/GimbalUav.h"
#include "Core/Transformer/Tree.h"
#include "Util/FPSCounter/FPSCounter.h"
#include "Util/ROS/Node.h"

int main(int argc, char* argv[]) {
    ros_util::init(argc, argv);

    //transformer::SetTranslation<CameraLink, GimbalLink>(-Eigen::Vector3d{ 118.05, 67.5, -41.7 });
    //transformer::SetTranslation<MuzzleLink, GimbalLink>(-Eigen::Vector3d{ 69.4, 67.5, 0 });
//    transformer::SetTranslation<GimbalGyro, CameraGyro>(Eigen::Vector3d{105.8, 0, -40.5} / 1000.0);
//    transformer::SetTranslation<GimbalGyro, MuzzleGyro>(Eigen::Vector3d{29.69, 0, 5.5} / 1000.0);

	for (int restartTime = 0;; ++restartTime) {
        LOG(INFO) << "Program started.";
		try {
            if (parameters::GimbalType == GimbalType::Infantry) {
                GimbalInfantry gimbal;
                gimbal.Always();
            }
            else if (parameters::GimbalType == GimbalType::Uav) {
                GimbalUav gimbal;
                gimbal.Always();
            }
            else if (parameters::GimbalType == GimbalType::Sentry) {

            }
		}
		catch (const std::exception& e) {
			LOG(ERROR) << "Uncaught " << typeid(e).name() << ": " << e.what();
		}
        catch (const char* str) {
            LOG(ERROR) << "Uncaught const char* : " << str;
        }
		catch (...) {
			LOG(ERROR) << "Uncaught unknown error";
		}
		LOG(ERROR) << "Program crashed, will restart in 5 seconds... ";
		std::this_thread::sleep_for(std::chrono::seconds(5));
	}
	return 0;
}

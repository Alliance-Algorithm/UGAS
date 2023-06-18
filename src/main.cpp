#include <chrono>
#include <thread>

#include "Control/Gimbal/Gimbal.h"
#include "Util/FPSCounter/FPSCounter.h"


int main() {
	for (int restartTime = 0;; ++restartTime) {
        LOG(INFO) << "Program started.";
		try {
			Gimbal gimbal;
			gimbal.Always();
		}
		catch (std::exception& e) {
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

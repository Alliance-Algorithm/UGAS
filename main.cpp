#define _CRT_SECURE_NO_WARNINGS

#include <chrono>
#include <thread>

#include "Control/Gimbal/Gimbal.h";
#include "Util/Debug/DebugCanvas.h"
#include "Util/Debug/DebugSettings.h"
#include "Util/FPSCounter/FPSCounter.h"


int main() {
	/*UGAS* ugas;
	for (int restartTime = 0;; ++restartTime) {
		try {
			LOG(INFO) << "Program starts, count " << restartTime;
			ugas = new UGAS();
			ugas->initial();
			ugas->always();
			delete ugas;
		}
		catch (std::exception& e) {
			LOG(ERROR) << "Uncaught " << typeid(e).name() << ">: " << e.what();
		}
		catch (...) {
			LOG(ERROR) << "Uncaught unknown error";
		}
		LOG(ERROR) << "Program crashed, will restart in 5 seconds... ";
		std::this_thread::sleep_for(std::chrono::seconds(5));
	}*/
	for (int restartTime = 0;; ++restartTime) {
		try {
			Gimbal gimbal;
			gimbal.Always();
		}
		catch (std::exception& e) {
			LOG(ERROR) << "Uncaught " << typeid(e).name() << ": " << e.what();
		}
		catch (...) {
			LOG(ERROR) << "Uncaught unknown error";
		}
		LOG(ERROR) << "Program crashed, will restart in 5 seconds... ";
		std::this_thread::sleep_for(std::chrono::seconds(5));
	}
	return 0;
}

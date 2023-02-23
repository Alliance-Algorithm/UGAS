#define _CRT_SECURE_NO_WARNINGS
#include "UGAS.h"

UGAS ugas;

int main() {
	for (int restartTime = 0;; ++restartTime) {
		try {
			LOG(INFO) << "Program starts, count " << restartTime;
			ugas.initial();
			ugas.always();
		}
		catch (std::exception& e) {
			LOG(ERROR) << "Uncaught " << typeid(e).name() << ">: " << e.what();
		}
		catch (...) {
			LOG(ERROR) << "Uncaught unknown error";
		}
		LOG(ERROR) << "Program crashed, will restart in 5 seconds... ";
		std::this_thread::sleep_for(std::chrono::seconds(5));
	}
	return 0;
}

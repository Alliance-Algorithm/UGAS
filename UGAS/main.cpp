#define _CRT_SECURE_NO_WARNINGS
#include "UGAS.h"

UGAS ugas;

int main() {
	//初始化日志库
	LOG_INIT();

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
		LOG(ERROR) << "Program crashed, restarting... ";
	}
	return 0;
}
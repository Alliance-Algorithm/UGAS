#include "Log.h"

INITIALIZE_EASYLOGGINGPP

void LOG_INIT() {
	try {
		//throw std::exception();
	}
	catch (std::exception& e) {
		std::cout << "Logger初始化时出现错误：" << typeid(e).name() << std::endl;
		std::cout << e.what() << std::endl;
		throw;
	}
}

#define _CRT_SECURE_NO_WARNINGS
#include "UGAS.h"

UGAS ugas;

int main() {
	try {
		ugas.initial();
		ugas.always();
	}
	catch (...) {
		// 自我重启

		// 还没实现haha

	}
	return 0;
}
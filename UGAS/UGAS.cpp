#include "UGAS.h"

UGAS::UGAS() :_com(*new GIMBAL_SERIAL()) {
	_com.Open(SERIAL_PORT);
	_com.RecvGimbalData();
	ParametersInit(static_cast<Team>(_com.team));
}

UGAS::~UGAS() {
	delete& _com;
}

void UGAS::initial() {

}

void UGAS::always() {

}

#pragma once

#include <iostream>
#include <eigen3/Eigen/Dense>
#include "Core/Identifier/Buff/BuffStruct.h"

using namespace std;
using namespace Eigen;

class EKF
{
public:
	EKF() { _initialized = false; }
	EKF(BuffAngularSpeed x) { initialization(x); }
	void initialization(BuffAngularSpeed x)
	{
		_x << x.speed, 0.913, 1.942, 0, 1.177; 
		setQ(); setP(); setH(); setR();
		_initialized = true;
		first_TimeStamp = x.timeStamp;
		_t=.0;
	}

	void Reset() { _initialized = false; }
	double Predict(double delta_time);
	double correct(BuffAngularSpeed s);
	TimeStamp first_TimeStamp;

	double PredictDeltaAngle(TimeStamp now, TimeStamp predict_timeStamp) const;//����now��Ԥ��ʱ�̵Ľ�λ��
	double PredictSpeed(TimeStamp now, TimeStamp predict_timeStamp);//����Ԥ��ʱ�̵Ľ��ٶ�

private:
	bool _initialized;
	double _t;
	Vector<double, 5> _x;// Spd A B C D
	Matrix<double, 5, 5> _F;
	Matrix<double, 5, 5> _P;
	Matrix<double, 5, 5> _Q;
	Matrix<double, 1, 5> _H;
	Matrix<double, 1, 1> _R;
	
	void setF(double delta_time);
	void setP();
	void setQ();
	void setH();
	void setR();
};
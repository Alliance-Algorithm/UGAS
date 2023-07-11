#include "EKF.h"

//set P,Q,H,R均只需初始化时调用一次
void EKF::setP()
{
	_P << 1, 0, 0, 0, 0,
		0, 1, 0, 0, 0,
		0, 0, 1, 0, 0,
		0, 0, 0, 1, 0,
		0, 0, 0, 0, 1;
}
//
void EKF::setQ()
{
	_Q << 0.001, 0, 0, 0, 0,
		0, 0.00001, 0, 0, 0,
		0, 0, 0.0001, 0, 0,
		0, 0, 0,0.00001, 0,
		0, 0, 0, 0, 0.00001;
	
	/*_Q << Q, 0, 0, 0, 0, 0,
	    0, Q, 0, 0, 0, 0,
		0, 0, Q, 0, 0, 0,
		0, 0, 0, Q, 0, 0,
		0, 0, 0, 0, Q, 0,
		0, 0, 0, 0, 0, Q;*/
}
//线性化的H
void EKF::setH()
{
	_H << 1, 0, 0, 0, 0;
}
//需要调（不会调）
void EKF::setR()
{
	double R = 0.888;//
	_R << R;
	//_R << R, 0,
	//	0, R;
}

//线性化的F
void EKF::setF(double delta_time)
{
	double s1 = sin(_x(2) * delta_time);
	double s2 = sin(_x(2) * _t + _x(3));
	double c1 = cos(_x(2) * delta_time);
	double c2 = cos(_x(2) * _t + _x(3));

	_F << c1, s1* c2, s1* delta_time* (_x(4) - _x(0)) + _x(1) * (delta_time * c1 * c2 - _t * s1 * s2),
		-1.0 * _x(1) * s1 * s2, 1.0 - c1,
		0, 1, 0, 0, 0,
		0, 0, 1, 0, 0,
		0, 0, 0, 1, 0,
		0, 0, 0, 0, 1;
}

double EKF::Predict(double delta_time)
{
	return _x(1) * sin(_x(2) * (_t + delta_time) + _x(3)) + _x(4);
}

double EKF::correct(BuffAngularSpeed s)
{
	if (!_initialized)
	{
		initialization(s);
	}
	else
	{
		//cout << s.timeStamp << endl;
		Vector<double, 1> z(s.speed);
		double new_t = 0.001 * (s.timeStamp - first_TimeStamp);
		//cout << new_t << endl;
		double delta_time = new_t - _t;
		//cout << delta_time << endl;
		//先验部分
		
		setF(delta_time);//F设置为线性化的F

		//cout << "F: " << endl << _F << endl;

		//更新_x的先验，直接用公式，不用线性化
		_x(0) = _x(1) * sin(_x(2) * new_t + _x(3)) + _x(4);
		_t = new_t;
		//_x(3) += _x(2) * delta_time;
		//cout << _x(5) << endl;
		MatrixXd Ft = _F.transpose();
		_P = _F * _P * Ft + _Q;

		//cout << "P: " << endl << _P << endl;

		//后验部分
		VectorXd y = z - _H * _x;//y非线性化，H线性化，此处等价
		//cout << "y: " << endl << y << endl;
		//cout << "z" << z << endl;
		//cout << "Hx" << _H * _x << endl;
		//cout << "x" << _x << endl;

		MatrixXd S = _H * _P * _H.transpose() + _R;
		//cout << "S: " << endl << S << endl;

		MatrixXd K = _P * _H.transpose() * S.inverse();
		//cout << "K: " << endl << K << endl;

		_x = _x + K * y;
		//cout << "x: " << endl << _x << endl;
		int size = _x.size();
		MatrixXd I = MatrixXd::Identity(size, size);
		_P = (I - K * _H) * _P;
		//cout << _x(5) << endl;
	}
	//cout << "EKF:" << _t << '\t' << _x(1) << "\t" << _x(2) << "\t" << _x(3) << "\t" << _x(4) << endl;
	return _x(0);
}


double EKF::PredictDeltaAngle(TimeStamp now, TimeStamp predict_timeStamp) const
{
	double t1 = 0.001 * now;
	double t2 = t1 + 0.001 * predict_timeStamp;
	double c1 = cos(_x(2) * t1 + _x(3));
	double c2 = cos(_x(2) * t2 + _x(3));
	double delta_angle = _x(1) / _x(2) * (c1 - c2) + _x(4) * predict_timeStamp;

	return delta_angle;
}

double EKF::PredictSpeed(TimeStamp now,TimeStamp predict_timeStamp)
{
	double t = (double)0.001 * (now + predict_timeStamp);
	return _x(1) * sin(_x(2) * t + _x(3)) + _x(4);
}
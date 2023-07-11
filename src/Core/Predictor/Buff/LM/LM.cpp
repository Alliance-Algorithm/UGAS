#include "LM.h"

using namespace Eigen;

constexpr double DERIV_STEP = 1e-6; // derivation accuracy
constexpr int MAX_ITER = 100; // maximum Iteration
constexpr double PI = 3.1415926535897;

double LM::error(const CircularQueue<BuffAngularSpeed, MaxData>& data, const VectorXd& params, int i)
{
	double x1 = params(0);
	double x2 = params(1);
	double x3 = params(2);
	double x4 = params(3);

	double t = (double)data.read(i).timeStamp * 0.001; // actual value (x)
	double f = data.read(i).speed; // actual value (y)

	return x1 * sin(x2 * t + x3) + x4 - f;
}

double LM::Error(const VectorXd& obj)
{
	return obj.squaredNorm() / 2;
}

VectorXd LM::objError(const CircularQueue<BuffAngularSpeed, MaxData>& data, const VectorXd& params)
{
	VectorXd obj(_total_data);
	for (int i = 0; i < _total_data; i++)
		obj(i) = error(data, params, i);
	return obj;
}

// numerical value derivative
double LM::Deriv(const CircularQueue<BuffAngularSpeed, MaxData>& data, int obj_i, const VectorXd& params, int params_i)
{
	VectorXd para1 = params;
	VectorXd para2 = params;

	para1(params_i) -= DERIV_STEP;
	para2(params_i) += DERIV_STEP;

	double obj1 = error(data, para1, obj_i);
	double obj2 = error(data, para2, obj_i);

	return (obj2 - obj1) / (2 * DERIV_STEP);
}

MatrixXd LM::Jacobin(const CircularQueue<BuffAngularSpeed, MaxData>& data, const VectorXd& params)
{
	int rowNum = _total_data;
	int colNum = params.rows();

	MatrixXd Jac(rowNum, colNum);

	for (int i = 0; i < rowNum; i++)
	{
		for (int j = 0; j < colNum; j++)
		{
			Jac(i, j) = Deriv(data, i, params, j);
		}
	}
	return Jac;
}

double LM::maxMatrixDiagonale(const MatrixXd& Hessian)
{
	int max = 0;
	for (int i = 0; i < Hessian.rows(); i++)
	{
		if (Hessian(i, i) > max)
			max = Hessian(i, i);
	}
	return max;
}

double LM::linerDeltaL(const VectorXd& step, const VectorXd& gradient, const double u)
{
	return step.transpose() * (u * step - gradient);
}

BuffFitData LM::Fitting()
{
	BuffFitData thisBuffFitData;
	thisBuffFitData.FitTime = _data.last().timeStamp;

	VectorXd obj = objError(_data, _paramsToBeFitted);
	MatrixXd Jac = Jacobin(_data, _paramsToBeFitted);
	MatrixXd Hessian = Jac.transpose() * Jac;
	VectorXd gradient = Jac.transpose() * obj;

	// _paramsToBeFitted[2] = 0;

	// parameters
	double tao = 0.1;
	long long v = 2;
	double eps1 = 1e-12, eps2 = 1e-12;
	double u = tao * maxMatrixDiagonale(Hessian);
	bool found = gradient.norm() <= eps1;
	if (!found) {

		double last_sum = 0;
		int iterCnt = 0;

		while (iterCnt < MAX_ITER)
		{
			obj = objError(_data, _paramsToBeFitted);
			Jac = Jacobin(_data, _paramsToBeFitted);
			Hessian = Jac.transpose() * Jac;
			gradient = Jac.transpose() * obj;

			// loss small enough
			if (gradient.norm() <= eps1)
			{
				//std::cout << "loss small enough" << std::endl;
				break;
			}

			//cout << "A: " << endl << A << endl;

			VectorXd step = (Hessian + u * MatrixXd::Identity(_num_params, _num_params)).inverse() * gradient;

			//std::cout << "step: " << std::endl << step << std::endl;

			// the amount of change is very small
			if (step.norm() <= eps2 * (_paramsToBeFitted.norm() + eps2))
			{
				//std::cout << "change small enough" << std::endl << std::endl;
				break;
			}

			VectorXd paramsNew(_paramsToBeFitted.rows());
			paramsNew = _paramsToBeFitted - step;

			// loss from last iteration
			obj = objError(_data, _paramsToBeFitted);

			// new loss
			VectorXd obj_new = objError(_data, paramsNew);

			// change damping
			double deltaF = Error(obj) - Error(obj_new);
			double deltaL = linerDeltaL(-1 * step, gradient, u);

			double roi = deltaF / deltaL;
			//cout << "roi is : " << roi << endl;
			if (roi > 0)
			{
				_paramsToBeFitted = paramsNew;
				u *= (std::max)(1.0 / 3.0, 1 - pow(2 * roi - 1, 3));
				v = 2;
			}
			else
			{
				u = u * v;
				v = v * 2;
			}
			//cout << "u = " << u << " v = " << v << endl;

			iterCnt++;
			//std::cout << "iteration: " << iterCnt << " times" << std::endl << std::endl;
		}
	}
	_this_cost = abs(obj.norm());
	if (_total_data == MaxData_)
	{
		if (_this_cost <= _last_cost)
		{

			thisBuffFitData.A = _paramsToBeFitted[0];
			thisBuffFitData.B = _paramsToBeFitted[1];
			thisBuffFitData.C = _paramsToBeFitted[2];
			thisBuffFitData.D = _paramsToBeFitted[3];
			_lastBuffFitData = thisBuffFitData;
			_last_cost = _this_cost;
			//std::cout << "loss reduce" << std::endl;
		}
		else
		{
			_lastBuffFitData.FitTime = _data.last().timeStamp;
			//std::cout << "loss upper" << _this_cost << "\t" << _last_cost << std::endl;
		}
	}
	else
	{
		//thisBuffFitData.FitTime = _data.last().timeStamp;
		thisBuffFitData.A = _paramsToBeFitted[0];
		thisBuffFitData.B = _paramsToBeFitted[1];
		thisBuffFitData.C = _paramsToBeFitted[2];
		thisBuffFitData.D = _paramsToBeFitted[3];

		_lastBuffFitData = thisBuffFitData;
		_last_cost = _this_cost;
	}

	return _lastBuffFitData;
}


double LM::PredictDeltaAngle(TimeStamp now, TimeStamp predict_timeStamp) const 
{
	double t1 = 0.001 * now;
	double t2 = t1 + 0.001 * predict_timeStamp;
	double c1 = cos(_lastBuffFitData.B * t1 + _lastBuffFitData.C);
	double c2 = cos(_lastBuffFitData.B * t2 + _lastBuffFitData.C);
	double delta_angle = _lastBuffFitData.A / _lastBuffFitData.B * (c1 - c2) + _lastBuffFitData.D * 0.001 * predict_timeStamp;

	return delta_angle;
}

double LM::PredictSpeed(TimeStamp now, TimeStamp predict_timeStamp)
{
	double t = (double)0.001 * (now + predict_timeStamp);
	return _lastBuffFitData.A * sin(_lastBuffFitData.B * t + _lastBuffFitData.C) + _lastBuffFitData.D;
}

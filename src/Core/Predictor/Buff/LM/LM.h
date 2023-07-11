#pragma once
/*
Creation Date:2023/1/17
Latest Update:2023/1/18
Developer(s):
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- ��С�������
* �����嵥��
* CircularQueue�������һ��Ԫ�ػ�overflow��ʹ��empty���CircularQueue�Ƿ�Ϊ��
* �����UGAS.Filterʹ�ø���(n+5)�Ķ���(��СΪn+1����)
*/

#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include "Core/Identifier/Buff/BuffStruct.h"


class LM
{
private:
	static const int MaxData_ = 200;//������ݸ���
	static const int MaxData = MaxData_+5;//������ݸ���
	const int _num_params = 4;//���Ż���������
	int _total_data;//���ݸ���
	BuffFitData _lastBuffFitData;
	bool firstFit = true;
	CircularQueue<BuffAngularSpeed, MaxData> _data;
	Eigen::Vector4d _paramsToBeFitted;//���Ż�����
	double _last_cost, _this_cost;//��ʧֵ

	double error(const CircularQueue<BuffAngularSpeed, MaxData>& data, const Eigen::VectorXd& params, int i);
	double Error(const Eigen::VectorXd& obj);
	Eigen::VectorXd objError(const CircularQueue<BuffAngularSpeed, MaxData>& data, const Eigen::VectorXd& params);
	double Deriv(const CircularQueue<BuffAngularSpeed, MaxData>& data, int obj_i, const Eigen::VectorXd& params, int params_i);
	Eigen::MatrixXd Jacobin(const CircularQueue<BuffAngularSpeed, MaxData>& data, const Eigen::VectorXd& params);
	double maxMatrixDiagonale(const Eigen::MatrixXd& Hessian);
	double linerDeltaL(const Eigen::VectorXd& step, const Eigen::VectorXd& gradient, const double u);
public:
	//���δ���
	LM(){
		reset();
	}
	void reset() 
	{ 
		_total_data = 0;
		_data.clear();
		_paramsToBeFitted[0] = 0.913;
		_paramsToBeFitted[1] = 1.942;
		_paramsToBeFitted[2] = 0;
		_paramsToBeFitted[3] = 1.177;
		_last_cost = 1000000000;
		_this_cost = 0;
	};
	inline void push(BuffAngularSpeed BuffData)	{
		if (_total_data < MaxData_) {
			_data.push_back(BuffData);
			_total_data++;
		}
		else {
			_data.pop();
			_data.push_back(BuffData);
		}
	}
	BuffFitData Fitting();
	double PredictDeltaAngle(TimeStamp now, TimeStamp predict_timeStamp) const;//����now��Ԥ��ʱ�̵Ľ�λ��
	double PredictSpeed(TimeStamp now, TimeStamp predict_timeStamp);//����Ԥ��ʱ�̵Ľ��ٶ�
};

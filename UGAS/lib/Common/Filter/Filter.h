#pragma once
/*
Creation Date: 2022/11/24
Latest Update: 2022/11/24
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 提供通用滤波方法
*/
#include <opencv2/opencv.hpp>
#include <Common/UniversalStruct.h>

namespace filters {
	// #### Filter 虚基类
	template <class type_t> class Filter {
	public:
		virtual type_t Predict(type_t value) = 0;
		virtual void Reset() = 0;
	};

	typedef Filter<float> FilterF;
	typedef Filter<double> FilterLF;

	namespace linear {
		// #### 线性滤波器
		template <class type_t>
		class Linear :public Filter<type_t> {
		protected:
			type_t _saved;
			bool _initial;
		public:
			Linear() :_saved(), _initial(true) {}

			type_t Predict(type_t value) {
				if (_initial) {
					_initial = false;
					return _saved = value;
				}
				else return _saved = _saved * 0.9f + value * 0.1f;
			}
			void Reset() { _saved = type_t(); _initial = true; }
		};

		// ### 高阶低通线性滤波器
		template <class type_t>
		class Linear_S :public Filter<type_t> {
		protected:
			type_t _saved;
			bool _initial;
		public:
			Linear_S() :_saved(), _initial(true) {}

			type_t Predict(type_t value) {
				if (_initial) {
					_initial = false;
					return _saved = value;
				}
				else return _saved = _saved * 0.99f + value * 0.01f;
			}
			void Reset() { _saved = type_t();  _initial = true; }
		};

		// ### 均值线性滤波器
		template <class type_t, int num>
		class Linear_E :public Filter<type_t> {
		protected:
			CircularQueue<type_t, num + 5> _queue;
			type_t _sum;
		public:
			Linear_E() :_sum() {}

			type_t Predict(type_t value) {
				_queue.push_back(value);
				if (_queue.size() > num) {
					_sum -= _queue.first();
					_queue.pop();
				}
				return (_sum += value) / _queue.size();
			}
			void Reset() { _sum = type_t(); _queue.clear(); }
		};
	}

	namespace PID {
		// #### PID滤波器
		template <class type_t>
		class PDfilter :public Filter<type_t> {
		protected:
			double _Kp, _Kd;
			type_t _last, _lastErr;
			bool _initial;
		public:
			PDfilter(double Kp = .7, double Kd = .1) :
				_Kp(Kp), _Kd(Kd), _last(), _lastErr(), _initial(true) {}

			type_t Predict(type_t value) {
				if (_initial) { // 第一个值
					_initial = false;
					return _last = value;
				}
				else { // 被低通滤波的值
					type_t err = _last - value;
					type_t res = err * _Kp + (err - _lastErr) * _Kd;
					_lastErr = err;
					return _last = value + res;
				}
			}
			void Reset() { _last = _lastErr = type_t(); _initial = true; }
		};
	}

	namespace kalman {
		// #### 卡尔曼滤波器 一阶
		class KalmanF :public FilterF {
		protected:
			cv::KalmanFilter _filter;
		public:
			KalmanF() :_filter(1, 1, 0) {
				_filter.transitionMatrix = (cv::Mat_<float>(1, 1) << 1);
				setIdentity(_filter.measurementMatrix);
				setIdentity(_filter.processNoiseCov, cv::Scalar::all(1e-5));
				setIdentity(_filter.measurementNoiseCov, cv::Scalar::all(1e-1));
				setIdentity(_filter.errorCovPost, cv::Scalar::all(1));
			}

			float Predict(float value) {
				_filter.correct(cv::Mat(1, 1, CV_32F, cv::Scalar(value)));
				return _filter.predict().at<float>(0);
			}
			void Reset() { /* Reset  KalmanFilter */ }
		};

		// #### 卡尔曼滤波器 二阶
		class KalmanF2D :public FilterF {
		protected:
			cv::KalmanFilter _filter;
		public:
			KalmanF2D() :_filter(2, 1, 0) {
				_filter.transitionMatrix = (cv::Mat_<float>(2, 2) << 1, 1, 0, 1);
				setIdentity(_filter.measurementMatrix);
				setIdentity(_filter.processNoiseCov, cv::Scalar::all(1e-5));
				setIdentity(_filter.measurementNoiseCov, cv::Scalar::all(1e-1));
				setIdentity(_filter.errorCovPost, cv::Scalar::all(1));
			}

			float Predict(float value) {
				_filter.correct(cv::Mat(1, 1, CV_32F, cv::Scalar(value)));
				return _filter.predict().at<float>(0);
			}
		};
	}
}

using LinearLF = filters::linear::Linear<double>;
using Linear_S_LF = filters::linear::Linear_S<double>;
template<int num>
using Linear_E_LF = filters::linear::Linear_E<double, num>;
using PDfilterLF = filters::PID::PDfilter<double>;
using filters::kalman::KalmanF;
using filters::kalman::KalmanF2D;

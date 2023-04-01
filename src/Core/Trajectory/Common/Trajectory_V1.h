#pragma once

#include <cmath>

#include <opencv2/opencv.hpp>

#include "Util/TimeStamp/TimeStampCounter.h"
#include "Control/Gimbal/Gimbal.h"
#include "Util/Parameter/Parameters.h"

#include "Core/Predictor/Armor\ArmorPredictor_V1.h"

class Trajectory_V1 {
private:
	//静靶
	double State_traget(const cv::Point3f tragetPoint, float& pitch) const {
		double dis_x = sqrt(tragetPoint.x * tragetPoint.x + tragetPoint.y * tragetPoint.y) / 1000;//水平距离
		double dis_h = tragetPoint.z / 1000;//高度，坐标轴待统一
		double now_pitch = atan(dis_h / dis_x);
		double res = tragetPoint.z / 1000;
		double temp_h = tragetPoint.z / 1000;//目前瞄准的高度
		double k1 = 0.13;//17mm
		//double k2 = 0.06;//42mm
		const double speed = 13.0f; // com.Get().speed;
		double flytimeSec = 0;
		double temp_time = (exp(k1 * dis_x) - 1) / k1 / speed;
		while (res > 1.)//允许的误差，待调
		{
			flytimeSec = temp_time / cos(now_pitch);
			double now_h = speed * sin(now_pitch) * flytimeSec - MathConsts::G * flytimeSec * flytimeSec / 2;
			res = dis_h - now_h;
			temp_h += res;
			now_pitch = atan(temp_h / dis_x);
		}
		flytimeSec = temp_time / cos(now_pitch);
		pitch = now_pitch;
		return flytimeSec;
	}
public:

	template <typename TargetType>
	GimbalAttitude GetShotAngle(const TargetType& target) const {
		GimbalAttitude attitude;

		cv::Point3f position = target.position;
		double flyTimeSec = State_traget(position, attitude.pitch);

		for (int i = Trajc_iterate; i-- > 0;) {
			position = target.Predict(flyTimeSec);
			flyTimeSec = State_traget(position, attitude.pitch);
			std::cout << attitude.pitch << std::endl;
		}

		std::cout << target.position << ' ' << attitude << std::endl;
		
		attitude.pitch -= target.gimbalAttitude.pitch;
		attitude.yaw = atan2(target.position.y, target.position.x) * 180 / MathConsts::Pi; // -target.gimbalAttitude.yaw;

		return attitude;
		// cv::circle(debugImg, _2Dposition, 5, COLOR_RED, 2);
	}
};

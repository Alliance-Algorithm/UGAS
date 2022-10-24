#pragma once
/*
Creation Date: 2022/10/19
Latest Update: 2022/10/19
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 通过FEM(有限元模拟)的方式模拟弹道，解算目标角度
*/
#include "../Trajectory.h"

class Trajectory_FEM : public Trajectory {
private:
	// 分析以angle角度到达distance时的z轴高度
	double Analyze(double distance, double angle, double altitudeTarget, double& flyTime);
	// 迭代弹丸飞行时间预测目标运动量
	void Iterate(cv::Point3f position, double& pitch, double& flyTime);
public:
	Trajectory_FEM(serial::GimbalSerial& com) :
		Trajectory(com) {}

	void GetShotAngle(const Target& target, TimeStamp ImgTime, double& yaw, double& pitch);
};

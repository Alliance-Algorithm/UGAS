#pragma once
/*
Creation Date: 2022/11/19
Latest Update: 2022/11/19
Developer(s): 21-THY
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 通过FEM(有限元模拟)的方式模拟弹道，解算目标角度
*/
#include "../Trajectory.h"

class Trajectory_FEM : public Trajectory {
private:
	cv::Point3f _3Dposition;
	cv::Point2f _2Dposition;

	// 分析以angle角度到达distance时的z轴高度
	double Analyze(double distance, double angle, double altitudeTarget, double& flyTime);
	// 迭代弹丸飞行时间预测目标运动量
	void Iterate(cv::Point3f position, double& pitch, double& flyTime);
public:
	void GetShotAngle(const int targetID, TimeStamp ImgTime, double& yaw, double& pitch);
};

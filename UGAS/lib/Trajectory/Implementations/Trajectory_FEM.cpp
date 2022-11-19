#include "Trajectory_FEM.h"
#include "Common/UniversalFunctions/UniversalFunctions.h"
#include "Common/TimeStamp/TimeStampCounter.h"
using namespace cv;

double Trajectory_FEM::Analyze(double distance, double angle, double altitudeTarget, double& flyTime) {
	Point2f position(0.0, 0.0);
	Point2f speed(com.Get().speed * cos(angle / 180 * PI), com.Get().speed * sin(angle / 180 * PI));
	int cnt = 0;
	while (position.x < distance) {
		if (position.y < altitudeTarget && speed.y < 0.0) break;
		Point2f acceleration(-Trajc_k * speed.x * speed.x, -Trajc_k * speed.y * speed.y - G);
		position += speed * Trajc_dertaT;
		speed += acceleration * Trajc_dertaT;
		if (++cnt > MAX_CNT) break;
	}
	flyTime = Trajc_dertaT * cnt;
	return position.y;
}

void Trajectory_FEM::Iterate(Point3f position, double& pitch, double& flyTime) {
	double distance = CoordinateDistance(position.x, position.z);
	double altitudeTarget = -position.y;
	double angleLow = angleLowest, angleHigh = angleHighest;
	while (angleHigh - angleLow > angleEPS) {
		double angleM = (angleLow + angleHigh) / 2;
		if (Analyze(distance, angleM, altitudeTarget, flyTime) > altitudeTarget)
			angleHigh = angleM;
		else angleLow = angleM;
	}
	pitch = angleHigh;
}

void Trajectory_FEM::GetShotAngle(const int targetID, TimeStamp ImgTime, double& yaw, double& pitch) {
	// 这里要解算Yaw需要将串口接收到的角度信息转换成真实世界的角度
	// 可是那还没写，这里暂时没有Yaw的动态瞄准
	yaw = 0.0;
	// Pitch的迭代解算
	double flyTime = 0.0;
	for (int i = Trajc_iterate; i; --i)
		Iterate(
			robots[targetID].Predict(
				TimeStampCounter::GetTimeStamp() - ImgTime + flyTime
			),
			pitch, flyTime);
}

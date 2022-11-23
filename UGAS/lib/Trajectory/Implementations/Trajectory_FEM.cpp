#include "Trajectory_FEM.h"
#include "Common/UniversalFunctions/UniversalFunctions.h"
#include "Common/TimeStamp/TimeStampCounter.h"
#include "Common/Color.h"
using namespace cv;

double Trajectory_FEM::EvaluateBulletFlightTime(const int targetID) {
	double res = 0.0f;
	const double speed = com.Get().speed;
	int cnt = iterations;
	while (cnt--) {
		Point3f hitPoint = robots[targetID].Predict(res);
		double distance = CoordinateDistance(hitPoint.x, hitPoint.z);
		double altitudeTarget = -hitPoint.y;
		double angleLow = angleLowest, angleHigh = angleHighest;
		while (angleHigh - angleLow > angleEPS) {
			double angleM = (angleLow + angleHigh) / 2;
			res = distance / speed / cos(angleHigh / 180 * PI);
			// 二次函数拟合(经典运动学) y = V0 * t - (1 / 2)G * t^2;
			double altitude = speed * sin(angleHigh / 180 * PI) * res - G * res * res / 2;
			if (altitude > altitudeTarget)
				angleHigh = angleM;
			else angleLow = angleM;
		}
	}
	return res;
}


double Trajectory_FEM::Analyze(double distance, double angle, double altitudeTarget, double& flyTime) {
	Point2f position(.0, .0);
	Point2f speed(com.Get().speed * cos(angle / 180 * PI), com.Get().speed * sin(angle / 180 * PI));
	int cnt = 0;
	while (position.x < distance) {
		if (position.y < altitudeTarget && speed.y < 0.0) break;
		Point2f acceleration(-Trajc_k * speed.x * speed.x, -Trajc_k * speed.y * speed.y - G);
		position += speed * Trajc_dertaT;
		speed += acceleration * Trajc_dertaT;
		if (++cnt > MAX_CNT) return DBL_MAX;
	}
	flyTime = Trajc_dertaT * cnt;
	return position.y;
}

void Trajectory_FEM::Iterate(Point3f position, double& pitch, double& flyTime) {
	double distance = CoordinateDistance(position.x, position.y);
	double altitudeTarget = -position.z;
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
	// Pitch差值的迭代解算
	double flyTime = EvaluateBulletFlightTime(targetID);
	for (int i = Trajc_iterate; i; --i)
		Iterate(
			_3Dposition = robots[targetID].Predict(staticReactionTime * 1000 +
				TimeStampCounter::GetTimeStamp() - ImgTime + flyTime
			),
		pitch, flyTime);
	pitch -= com.Get().pitchA;

	// Yaw差值的解算
	_2Dposition = PnPsolver.RevertPnP(_3Dposition);
	yaw = _2Dposition.x - (frameWidth >> 1);
	//LOG(INFO) << _2Dposition << '\n';

#if DEBUG_PREDICT == 1
	//*///
	static filters::PID::PDfilter<cv::Point2f> _2DpositionFilter;
	cv::circle(debugImg, _2DpositionFilter.Predict(_2Dposition), 5, COLOR_RED, 2);
	/*///
	cv::circle(debugImg, _2Dposition, 5, COLOR_RED, 2);
	//*///
#endif
}

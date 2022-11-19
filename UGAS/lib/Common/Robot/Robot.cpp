#include "Robot.h"
#include "Parameters.h"

enum RotateDirc { UNKNOWN = 0, LEFT, RIGHT };

Robot robots[10];

Robot::Robot() :_latestUpdate(0ULL),
	_pitch(.0), _roll(.0), _yaw(.0),
	_robotCenter(), _movingSpeed(), _armor(),
	_rotate(RotateDirc::UNKNOWN), _rotateSpeed(.0) {}

void Robot::Update(TimeStamp tp, ArmorPlate armor) {
	if (_latestUpdate != tp) {
		if(tp - _latestUpdate > keep_tracking * 1000)


		_armor = armor;
		_latestUpdate = tp;
	}
	else { // 更新观测的第二个装甲板

	}
}

cv::Point3f Robot::Predict(int millisec) {
	cv::Point3f prediction = _robotCenter;

	if (_rotate == RotateDirc::UNKNOWN) {
		prediction += static_cast<cv::Point3f>(_movingSpeed * millisec);
	}
	else {

		switch (_rotate) {
		case LEFT:

			break;
		case RIGHT:

			break;
		default: break;
		}
	}

}



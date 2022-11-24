#include "Robot.h"
#include "Common/PnP/PnP.h"
#include "Parameters.h"
#include "Common/UniversalFunctions/UniversalFunctions.h"

enum RotateDirc { UNKNOWN = 0, LEFT, RIGHT };

Robot robots[10];

Robot::Robot() :_latestUpdate(0ULL), _rotationLatestUpdate(0ULL),
	_robotCenter(), _movingSpeed(), _armor(), _armorCenter(),
	_rotate(RotateDirc::UNKNOWN), _rotateSpeed(.0), _possibility(.0) {}

void Robot::Update(TimeStamp ImgTime, const ArmorPlate& armor) {
	if (_latestUpdate != ImgTime) {
		if (ImgTime - _latestUpdate > keep_tracking * 1000)
		{ // 新观察到的目标
			_armorCenter = PnPsolver.SolvePnP(armor);
			_speedFilter.Reset();
			_movingSpeed = cv::Vec3f();
		}
		else
		{ // 持续跟踪的目标
			// 比较预测点与观测点的距离，保存预测点
			cv::Point2f prediction = PnPsolver.RevertPnP(
				this->Predict(ImgTime - _latestUpdate)
			);

#if DEBUG_IMG == 1 && DEBUG_TRACK == 1
			cv::circle(debugImg, prediction, 5, COLOR_YELLOW, 2);
#endif

			// 计算新的三维中心
			cv::Point3f lastPostion = _armorCenter;
			_armorCenter = PnPsolver.SolvePnP(armor);
			
			if (P2PDis(prediction, armor.center()) < maxArmorTrackDis)
			{ // 对同一个装甲板有效的跟踪则更新速度
				// 用封装好的滤波
				_movingSpeed = _speedFilter.Predict(
					static_cast<cv::Vec3f>(_armorCenter - lastPostion) /
					static_cast<double>(ImgTime - _latestUpdate)
				);
			}
			else if (_movingSpeed == cv::Vec3f())
			{ // 如果没有速度直接更新，加快对本身有一定初速度目标的响应
				_movingSpeed =
					static_cast<cv::Vec3f>(_armorCenter - lastPostion) /
					static_cast<double>(ImgTime - _latestUpdate);
			}
			// 如果超过最大跟踪距离，不认为是同一个装甲板，在可接受的时间差内继承速度
			else if (ImgTime - _latestUpdate > 100) // 这个值很容易调出问题暂时弄成定值
			{ // 超过一个较短时间即放弃对速度的继承
				_movingSpeed = cv::Vec3f();
				_speedFilter.Reset();
			}
		}
		// 暂时乱打一波
		_possibility = 100.;

		// 更新数据
		_armor = armor;
		_latestUpdate = ImgTime;
	}
	else { // 更新观测的第二个装甲板
		if (ImgTime - _rotationLatestUpdate > rotation_validity * 1000)
		{ // 新得到的旋转数据

		}
		else
		{ // 持续更新的旋转数据

		}

		_rotationLatestUpdate = ImgTime;
	}
}

double Robot::GetPossibility() {
	return (TimeStampCounter::GetTimeStamp() - _latestUpdate < keep_tracking * 1000) ? \
		_possibility : (_possibility = .0);
}

cv::Point3f Robot::Predict(int millisec) const {
	cv::Point3f prediction = _movingSpeed * millisec;

	// 没有有效的旋转信息则为动靶模式
	if (TimeStampCounter::GetTimeStamp() - _rotationLatestUpdate
			> rotation_validity * 1000 || _rotate == RotateDirc::UNKNOWN)
	{
		prediction += _armorCenter;
	}
	else { // 对于小陀螺的预测
		prediction += _robotCenter;
		switch (_rotate) {
		case LEFT:

			break;
		case RIGHT:

			break;
		default: break;
		}
	}
	return prediction;
}

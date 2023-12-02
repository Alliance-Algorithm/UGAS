#pragma once

#include <cmath>

#include <tuple>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "Core/Tracker/TrackerStruct.h"
#include "Util/Parameter/Parameters.h"
#include "Util/TimeStamp/TimeStampCounter.h"

class Trajectory_V1 {
public:
    /*! 获取射击角度
    * \return 返回云台偏移量，格式为tuple[yaw, pitch]，单位弧度制，遵循右手定则。
    */
    static std::tuple<double, double> GetShotAngle(const TargetInterface& target, const double& speed, bool predict_movement = true, double time_shift = 0) {
        std::tuple<double, double> result;
        auto& [yaw, pitch] = result;
        double fly_time = 0;
        GimbalGyro::Position pos;
        GetShotAngleInternal(target.Predict(time_shift), speed, yaw, pitch, fly_time);
        if (predict_movement)
            GetShotAngleInternal(pos = target.Predict( time_shift + fly_time), speed, yaw, pitch, fly_time);

        ros_util::PointBroadcast(pos);
        //std::cout << fly_time;
        return result;
    }

private:
    static void GetShotAngleInternal(const GimbalGyro::Position& target_pos, const double speed, double& yaw, double& pitch, double& fly_time) {
        MuzzleLink::DirectionVector shotVec = GetShotVector(target_pos, speed, fly_time);

        yaw = atan2(shotVec->y(), shotVec->x());
        pitch = -atan2(shotVec->z(), sqrt(shotVec->y() * shotVec->y() + shotVec->x() + shotVec->x()));
    }

    [[nodiscard]] static MuzzleGyro::DirectionVector GetShotVector(const MuzzleGyro::Position& target_pos, const double speed, double& fly_time) {
        // 不考虑空气阻力

        const double& x = target_pos->x();
        const double& y = target_pos->y();
        const double& z = target_pos->z();


        double yaw = atan2(y, x);
        double pitch = 0;

        double a = speed * speed;                  // v0 ^ 2
        double b = a * a;                          // v0 ^ 4
        double c = x * x + y * y;                  // xt ^ 2
        double d = c * c;                          // xt ^ 4
        double e = parameters::G * parameters::G;  // g ^ 2

        double xt = sqrt(c);                    // target horizontal distance

        double f = b * d * (b - e * c - 2 * parameters::G * a * z);
        if (f >= 0) {
            pitch = -atan((b * c - sqrt(f)) / (parameters::G * a * c * xt));
        }

        auto result = MuzzleGyro::DirectionVector{ cos(pitch) * cos(yaw), cos(pitch) * sin(yaw),-sin(pitch) };

        fly_time = xt / (cos(pitch) * speed);

        return result;
    }
};

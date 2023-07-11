#pragma once

#include <cmath>

#include <tuple>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "Control/Gimbal/Gimbal.h"
#include "Core/Transformer/SimpleTransformer.h"
#include "Util/Parameter/Parameters.h"
#include "Util/TimeStamp/TimeStampCounter.h"

class Trajectory_V1 {
public:
    /*! 获取射击角度，不做预测处理。
    * \return 返回云台偏移量，格式为tuple[yaw, pitch]，单位弧度制，遵循右手定则。
    */
    template <typename TransformerType>
    auto GetShotAngle(const Eigen::Vector3d& targetGimbalGyro, const double speed, const TransformerType& transformer) const {
        std::tuple<double, double> result;
        auto& [yaw, pitch] = result;

        Eigen::Vector3d shotVec = GetShotVector(transformer.GimbalGyro2MuzzleGyro(targetGimbalGyro), speed);
        shotVec = transformer.Gyro2Link(shotVec);

        yaw = atan2(shotVec.y(), shotVec.x());
        pitch = -atan2(shotVec.z(), sqrt(shotVec.y() * shotVec.y() + shotVec.x() + shotVec.x()));

        return result;
    }

    /*! 获取射击角度，还没做预测。
    * \return 返回云台偏移量，格式为tuple[yaw, pitch]，单位弧度制，遵循右手定则。
    */
    template <typename TargetType, typename TransformerType>
    auto GetShotAngle(const TargetType& target, const double speed, const TransformerType& transformer) const {
        return GetShotAngle(target.Predict(0), speed, transformer);
    }


    template <typename TargetType>
    auto GetShotAngle(const TargetType& target, const double speed) const {
        auto pos = target.Predict(0);


        auto transformer = SimpleTransformer(pos);
        return GetShotAngle(transformer.CameraLink2GimbalLink(transformer.Link2Gyro(pos)), speed, transformer);
    }

    // calc buff: trajectory with external gyroscope
    template<typename TransformerType>
    std::optional<GimbalAttitude> GetShotAngle(
        BuffPredictor_V1& buffPredictor,
        TransformerType transformer)
    {
        Eigen::Vector3d gimbal_gyro = buffPredictor.gimbal_gyro;
        //std::cout << "gimbal_gyro:\n" << gimbal_gyro << std::endl;

        GimbalAttitude attitude;
        //*
        {
            const double electricDelayTimeSec = 0.1; // offset data of Mecanum
            //const double electricDelayTimeSec = 0.15; // offset data of Omni

            cv::Point3f gimbal_gyro_cvpoint = cv::Point3f(gimbal_gyro(0), gimbal_gyro(1), gimbal_gyro(2));

            double traFlyTimeSec = StateTraget(gimbal_gyro_cvpoint, attitude.pitch);
            double delayTimeSec = traFlyTimeSec + electricDelayTimeSec;
            for (int i = Trajc_iterate; i-- > 0;) {
                if (auto&& gimbal_gyro_opt = buffPredictor.Predict(delayTimeSec, transformer, i == 0)) {
                    gimbal_gyro_cvpoint = cv::Point3f((*gimbal_gyro_opt)(0), (*gimbal_gyro_opt)(1), (*gimbal_gyro_opt)(2));
                    traFlyTimeSec = StateTraget(gimbal_gyro_cvpoint, attitude.pitch);
                    delayTimeSec = traFlyTimeSec + electricDelayTimeSec;
                    //std::cout << "flyTime = " << flyTimeSec << std::endl;
                }
                else {
                    //std::cout << "nullopt for buffPredictor.Predict()" << std::endl;
                    return std::nullopt;
                }
            }

            // yaw: left-, right+; pitch: down-, up+
            attitude = buffPredictor.CalcGimbalAttitude(gimbal_gyro_cvpoint);

            // display gyros attitude & target attitude
            if constexpr (debugCanvas.buff) {
                if (false) {
                    cv::Mat &debugImg = debugCanvas.buff.GetMat();
                    cv::line(debugImg, cv::Point(debugImg.cols / 2 - 15, debugImg.rows / 2),
                             cv::Point(debugImg.cols / 2 + 15, debugImg.rows / 2), COLOR_RED, 1);
                    cv::line(debugImg, cv::Point(debugImg.cols / 2, debugImg.rows / 2 - 15),
                             cv::Point(debugImg.cols / 2, debugImg.rows / 2 + 15), COLOR_RED, 1);
                    cv::circle(debugImg,
                               cv::Point(10 * attitude.yaw + debugCanvas.buff.GetMat().cols / 2,
                                         -10 * attitude.pitch + debugCanvas.buff.GetMat().rows / 2),
                               5, COLOR_RED, 3, 8);
                }
            }

            const float ylimit = 60, plimit = 30;
            // if(attitude.yaw>ylimit) attitude.yaw=ylimit;
            // else if(attitude.yaw<-ylimit) attitude.yaw=-ylimit;
            // if(attitude.pitch>plimit) attitude.pitch=plimit;
            // else if(attitude.pitch<-plimit) attitude.pitch=-plimit;
            if (attitude.yaw > ylimit || attitude.yaw < -ylimit) return std::nullopt;
            if (attitude.pitch > plimit || attitude.pitch < -plimit) return std::nullopt;
            if (attitude.yaw != attitude.yaw) return std::nullopt;
            if (attitude.pitch != attitude.pitch) return std::nullopt;


            // attitude.yaw *= -1;
            // attitude.pitch *= -1;
            // [Discard this line!] yaw: left+, right-; pitch: down+, up-

            // std::cout << "correct: " << buffPredictor.gimbal_gyro << " \t" << attitude << std::endl;
        }
        /*/
        {
            const double speed = 25.0f; // default speed

            Eigen::Vector3d shotVec = GetShotVector(transformer.GimbalGyro2MuzzleGyro(gimbal_gyro), speed);

            attitude.yaw = atan2(shotVec.y(), shotVec.x());
            attitude.pitch = -atan2(shotVec.z(), sqrt(shotVec.y() * shotVec.y() + shotVec.x() * shotVec.x()));


            //std::cout << "gimbal_gyro:\n" << gimbal_gyro << "\nshotVec:\n" << shotVec << "\nattitude: " << attitude << std::endl;

            // draw target delta attitude
            if constexpr (debugCanvas.buff) {
                cv::Mat& debugImg = debugCanvas.buff.GetMat();
                cv::line(debugImg, cv::Point(debugImg.cols / 2 - 15, debugImg.rows / 2),
                         cv::Point(debugImg.cols / 2 + 15, debugImg.rows / 2), COLOR_RED, 1);
                cv::line(debugImg, cv::Point(debugImg.cols / 2, debugImg.rows / 2 - 15),
                         cv::Point(debugImg.cols / 2, debugImg.rows / 2 + 15), COLOR_RED, 1);
                cv::circle(debugImg,
                           cv::Point(50 * attitude.yaw + debugCanvas.buff.GetMat().cols / 2,
                                     -50 * attitude.pitch + debugCanvas.buff.GetMat().rows / 2),
                           5, COLOR_RED, 3, 8);
            }
        }
        //*/

        return attitude;
    }

    // calc buff: trajectory with cboard gyroscope (no external gyroscope)
    std::optional<GimbalAttitude> GetShotAngle(
        BuffPredictor_V1& buffPredictor,
        const GimbalAttitude gyrosAttitude)
    {
        // gyrosAttitude has been corrected in serial when recieve it (l-r+ d-u+)
        //std::cout << gyrosAttitude.pitch << " " << gyrosAttitude.yaw << std::endl;

        GimbalAttitude attitude;
        const double electricDelayTimeSec = 0.1; // offset data of Mecanum
        //const double electricDelayTimeSec = 0.15; // offset data of Omni

        cv::Point3f position = buffPredictor.position;
        std::cout<<"your pitch to target:"<<attitude.pitch<<std::endl;
        double traFlyTimeSec = StateTraget(position, attitude.pitch);
        double flyTimeSec = traFlyTimeSec + electricDelayTimeSec;
        //std::cout << "TraFlyTime = " << traFlyTimeSec << " MachanicalDelayTime = " << machanicalDelayTimeSec << std::endl;

        for (int i = Trajc_iterate; i-- > 0;) {
            if (auto&& position_opt = buffPredictor.Predict(flyTimeSec, gyrosAttitude, i == 0)) {
                position = *position_opt;
                traFlyTimeSec = StateTraget(position, attitude.pitch);
                flyTimeSec = traFlyTimeSec + electricDelayTimeSec;
                //std::cout << "flyTime = " << flyTimeSec << std::endl;
            }
            else {
                //std::cout << "nullopt fot buffPredictor.Predict()" << std::endl;
                return std::nullopt;
            }
        }

        // yaw: left-, right+; pitch: down-, up+
        attitude = buffPredictor.CalcGimbalAttitude(position);

        // display gyros attitude & target attitude
        if constexpr (true && debugCanvas.buff) {
            cv::Mat& debugImg = debugCanvas.buff.GetMat();
            cv::line(debugImg, cv::Point(debugImg.cols / 2 - 15, debugImg.rows / 2),
                cv::Point(debugImg.cols / 2 + 15, debugImg.rows / 2), COLOR_RED, 1);
            cv::line(debugImg, cv::Point(debugImg.cols / 2, debugImg.rows / 2 - 15),
                cv::Point(debugImg.cols / 2, debugImg.rows / 2 + 15), COLOR_RED, 1);
            cv::circle(debugImg,
                cv::Point(10 * gyrosAttitude.yaw + debugCanvas.buff.GetMat().cols / 2,
                    -10 * gyrosAttitude.pitch + debugCanvas.buff.GetMat().rows / 2),
                5, COLOR_PINK, 3, 8);
            cv::circle(debugImg,
                cv::Point(10 * attitude.yaw + debugCanvas.buff.GetMat().cols / 2,
                    -10 * attitude.pitch + debugCanvas.buff.GetMat().rows / 2),
                5, COLOR_RED, 3, 8);
        }
        //std::cout << "raw: " << buffPredictor.position << " \t" << attitude << std::endl;

        // [same] yaw: left - , right + ; pitch: down - , up +
        // const float compensationRaito = 1.0f;
        // attitude.yaw += compensationRaito * (attitude.yaw - gyrosAttitude.yaw);
        // attitude.pitch += compensationRaito * (attitude.yaw - gyrosAttitude.yaw);

        const float ylimit = 60, plimit = 30;
        // if(attitude.yaw>ylimit) attitude.yaw = ylimit;
        // else if(attitude.yaw<-ylimit) attitude.yaw = -ylimit;
        // if(attitude.pitch>plimit) attitude.pitch = plimit;
        // else if(attitude.pitch<-plimit) attitude.pitch= -plimit;
        if (attitude.yaw > ylimit || attitude.yaw < -ylimit) return std::nullopt;
        if (attitude.pitch > plimit || attitude.pitch < -plimit) return std::nullopt;
        if (attitude.yaw != attitude.yaw) return std::nullopt;
        if (attitude.pitch != attitude.pitch) return std::nullopt;


        //attitude.yaw *= -1;
        //attitude.pitch *= -1;
        // [Discard this line!] yaw: left+, right-; pitch: down+, up-

        //std::cout << "correct: " << buffPredictor.position << " \t" << attitude << std::endl;

        return attitude;
    }

private:
    Eigen::Vector3d GetShotVector(const Eigen::Vector3d& targetGimbalGyro, const double speed) const {
        // 不考虑空气阻力

        double x = targetGimbalGyro.x() / 1000;
        double y = targetGimbalGyro.y() / 1000;
        double z = targetGimbalGyro.z() / 1000;

        double hDis = sqrt(x * x + y * y);

        double yaw = atan2(y, x);
        double pitch = 0;

        double a = speed * speed;                  // v0 ^ 2
        double b = a * a;                          // v0 ^ 4
        double c = hDis * hDis;                    // xt ^ 2
        double d = c * c;                          // xt ^ 4
        double e = MathConsts::G * MathConsts::G;  // g ^ 2

        double f = b * d * (b - e * c - 2 * MathConsts::G * a * z);
        if (f >= 0) {
            pitch = -atan((b * c - sqrt(f)) / (MathConsts::G * a * c * hDis));
        }

        return { cos(pitch) * cos(yaw), cos(pitch) * sin(yaw),-sin(pitch) };
    }

    double StateTraget(const cv::Point3f tragetPoint, float& pitch) const {
        double dis_x = sqrt(tragetPoint.x * tragetPoint.x + tragetPoint.y * tragetPoint.y) / 1000;//水平距离
        double dis_h = tragetPoint.z / 1000;//高度，坐标轴待统一
        double now_pitch = atan(dis_h / dis_x);
        double res = 1e5;
        double temp_h = tragetPoint.z / 1000;//目前瞄准的高度
        double k1 = 0.038; //17mm
        //double k2 = 0.06; //42mm
        const double speed = 25.0f; // com.Get().speed; warning: didn't change
        double flytimeSec = 0;
        double temp_time = (exp(k1 * dis_x) - 1) / k1 / speed;
        int iteration = 30;
        while (--iteration != 0 && res > 1e-3)//允许的误差，待调
        {
            flytimeSec = temp_time / cos(now_pitch);
            double now_h = speed * sin(now_pitch) * flytimeSec - MathConsts::G * flytimeSec * flytimeSec / 2;
            res = dis_h - now_h;
            temp_h += res;
            now_pitch = atan(temp_h / dis_x);
        }
        flytimeSec = temp_time / cos(now_pitch);
        pitch = now_pitch * 180 / MathConsts::Pi;
        return flytimeSec;
    }
};

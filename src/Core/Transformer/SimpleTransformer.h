#pragma once
/*
Creation Date: 2023/05/26
Latest Update: 2023/05/26
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
最基础的坐标系转换器，不使用陀螺仪，而是理想地认为相机光心水平于目标装甲板中心，理想地认为相机光心、云台中心和弹丸发射点重合。
*/

#include <eigen3/Eigen/Dense>


class SimpleTransformer {
public:
    /*! 传入相机坐标系下的装甲板三维坐标，通过理想地认为相机光心水平于目标装甲板中心，计算出云台当前的pitch轴角度，用于坐标转换。
    * 注意：不同于IMU得到的Transformer每帧只有一个实例，一般对于每一个识别到的装甲板，都应该使用其对应实例化的SimpleTransformer进行坐标转换。
    * \param target 相机坐标系下的装甲板三维坐标
    */
    explicit SimpleTransformer(const Eigen::Vector3d& target) {
        pitch = -atan2(target.z(), target.x());
    }

    Eigen::Vector3d CameraLink2GimbalLink(const Eigen::Vector3d& srcPos) const {
        return srcPos;
    }

    Eigen::Vector3d Link2Gyro(const Eigen::Vector3d& srcPos) const {
        return {
            srcPos.x() * cos(pitch) - srcPos.z() * sin(pitch),
            srcPos.y(),
            srcPos.z() * cos(pitch) + srcPos.x() * sin(pitch)
        };
    }

    Eigen::Vector3d Gyro2Link(const Eigen::Vector3d& srcPos) const {
        return {
            srcPos.x() * cos(pitch) + srcPos.z() * sin(pitch),
            srcPos.y(),
            srcPos.z() * cos(pitch) - srcPos.x() * sin(pitch)
        };
    }

    Eigen::Vector3d GimbalGyro2MuzzleGyro(const Eigen::Vector3d& srcPos) const {
        return srcPos;
    }

private:
    // 计算出的假想云台俯仰姿态，方向遵循右手定则
    double pitch;
};
#pragma once
/*
Creation Date: 2023/05/26
Latest Update: 2023/05/26
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
用于单机调试，接口与CBoardInfantry相同，但无实际操作。
*/

#include "Util/Parameter/Parameters.h"

class VirtualCBoard {
public:
    VirtualCBoard(bool debugOutput = false) : _debugOutput(debugOutput) { }

    ~VirtualCBoard() {    }

    /*! 向除哨兵外的地面兵种发送云台瞄准数据
    * \param yaw pitch 单位使用弧度制，方向遵循右手定则
    */
    void Send(double yaw, double pitch) {
        if (_debugOutput) {
            yaw *= 180.0 / MathConsts::Pi;
            pitch *= 180.0 / MathConsts::Pi;
            std::cout << "Send: [" << yaw << ", " << pitch << "] deg\n";
        }
    }

    /*! 向无人机发送云台瞄准数据
    * \param yaw pitch 单位使用弧度制，方向遵循右手定则
    */
    void SendUAV(double yaw, double pitch) {
        if (_debugOutput) {
            yaw *= 180.0 / MathConsts::Pi;
            pitch *= 180.0 / MathConsts::Pi;
            std::cout << "SendUAV: [" << yaw << ", " << pitch << "] deg\n";
        }
    }

    void Receive() { }

    ArmorColor GetEnemyColor() {
        return Parameters::DefaultEnemyColor;
    }

    float GetBulletSpeed() {
        return Parameters::DefaultBulletSpeed;
    }

private:
    bool _debugOutput;
};
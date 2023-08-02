#pragma once
/*
Creation Date: 2023/05/26
Latest Update: 2023/08/02
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
用于单机调试，接口与CBoardInfantry相同，但无实际操作。
*/

#include "Util/Parameter/Parameters.h"

class VirtualCBoard {
public:
    VirtualCBoard(bool debugOutput = false) : _debugOutput(debugOutput) { }

    ~VirtualCBoard() = default;

    // 发送云台瞄准数据
    // 没有目标时调用
    void Send() const {
        if (_debugOutput) {
            std::cout << "Send: [0, 0] deg\n";
        }
    }

    // 发送云台瞄准数据
    // yaw pitch: 击中目标所需云台移动差值，单位使用弧度制，方向遵循右手定则
    // fire: 保留
    void Send(double yaw, double pitch, bool fire) const {
        if (_debugOutput) {
            yaw *= 180.0 / parameters::Pi;
            pitch *= 180.0 / parameters::Pi;
            std::cout << "Send: [" << yaw << ", " << pitch << "] deg\n";
        }
    }

    void Receive() const { }

    [[nodiscard]] ArmorColor get_enemy_color() const {
        return parameters::DefaultEnemyColor;
    }

    [[nodiscard]] double get_bullet_speed() const {
        return parameters::DefaultBulletSpeed;
    }

    [[nodiscard]] bool get_auto_scope_enabled() const {
        return true;
    }

private:
    bool _debugOutput;
};


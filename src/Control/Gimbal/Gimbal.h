#pragma once

#include <iostream>

struct GimbalAttitude {
    float yaw, pitch;
    GimbalAttitude() : yaw(0), pitch(0) { }
    GimbalAttitude(float yaw, float pitch) : yaw(yaw), pitch(pitch) { }
    friend std::ostream& operator<<(std::ostream&, const GimbalAttitude&);
};

inline std::ostream& operator<<(std::ostream& os, const GimbalAttitude& attitude) {
    os << "[yaw=" << attitude.yaw << ", pitch=" << attitude.pitch << ']';
    return os;
}


class Gimbal {

public:

    Gimbal() { }

    void Always();
};

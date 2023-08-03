#pragma once
/*
Creation Date: 2023/07/21
Latest Update: 2023/07/22
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 定义tf tree
*/

#include "Util/Parameter/Parameters.h"
#include "Core/Transformer/Transformer.h"


inline transformer::Translation gimbal_camera_translation_ = transformer::Translation::Identity();
inline transformer::Translation gimbal_muzzle_translation_ = transformer::Translation::Identity();


struct GimbalGyro : transformer::Frame<GimbalGyro> {
    using transformer::Frame<GimbalGyro>::Frame;

    using Header = typename transformer::Root;
    static constexpr char name[] = "gimbal_gyro";
};

struct CameraGyro : transformer::Frame<CameraGyro> {
    using transformer::Frame<CameraGyro>::Frame;

    using Header = GimbalGyro;
    static constexpr auto& transform = parameters::TranslationGimbalToCamera;
    static constexpr char name[] = "camera_gyro";
};

struct MuzzleGyro : transformer::Frame<MuzzleGyro> {
    using transformer::Frame<MuzzleGyro>::Frame;

    using Header = GimbalGyro;
    static constexpr auto& transform = parameters::TranslationGimbalToMuzzle;
    static constexpr char name[] = "muzzle_gyro";
};

struct TransmitterGyro : transformer::Frame<TransmitterGyro> {
    using transformer::Frame<TransmitterGyro>::Frame;

    using Header = GimbalGyro;
    static constexpr auto& transform = parameters::TranslationGimbalToTransmitter;
    static constexpr char name[] = "Transmitter_gyro";
};


struct GimbalLink : transformer::Frame<GimbalLink> {
    using transformer::Frame<GimbalLink>::Frame;

    using Header = GimbalGyro;
    static inline transformer::Rotation transform = transformer::Rotation::Identity();
    static constexpr char name[] = "gimbal_link";
};

struct CameraLink : transformer::Frame<CameraLink> {
    using transformer::Frame<CameraLink>::Frame;

    using Header = GimbalLink;
    static constexpr auto& transform = parameters::TranslationGimbalToCamera;
    static constexpr char name[] = "camera_link";
};

struct MuzzleLink : transformer::Frame<MuzzleLink> {
    using transformer::Frame<MuzzleLink>::Frame;

    using Header = GimbalLink;
    static constexpr auto& transform = parameters::TranslationGimbalToMuzzle;
    static constexpr char name[] = "muzzle_link";
};

struct TransmitterLink : transformer::Frame<TransmitterLink> {
    using transformer::Frame<TransmitterLink>::Frame;

    using Header = GimbalLink;
    static constexpr auto& transform = parameters::TranslationGimbalToTransmitter;
    static constexpr char name[] = "Transmitter_link";
};

#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_core/msgs.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>

#include "Control/Gimbal/GimbalInfantry.h"
#include "Core/Tracker/TrackerStruct.h"
#include "Core/Trajectory/Common/Trajectory_V1.h"

namespace ugas {

class Component
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Component()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , target_(nullptr) {
        ros_util::init();

        register_input("/predefined/update_count", update_count_);
        register_input("/tf", tf_);
        register_input("/robot_color", color_);
        register_input("/robot_id", robot_id_);
        register_input("/auto_rune", buff_mode_);
        register_output(
            "/gimbal/auto_aim/control_direction", control_direction_, Eigen::Vector3d::Zero());

        gimbal_ = std::make_unique<GimbalInfantry>();

        exposure_time_          = get_parameter("exposure_time").as_int();
        armor_predict_duration_ = get_parameter("armor_predict_duration").as_int();
        buff_predict_duration_  = get_parameter("buff_predict_duration").as_int();
        yaw_error               = get_parameter("yaw_error").as_double();
        pitch_error             = get_parameter("pitch_error").as_double();
    }

    ~Component() {
        if (gimbal_thread_.joinable())
            gimbal_thread_.join();
    }

    void update() override {
        if (*update_count_ == 0) {
            gimbal_thread_ = std::thread{[this]() {
                gimbal_->Always(
                    target_, timestamp_, color_, robot_id_,
                    std::chrono::milliseconds(exposure_time_), buff_mode_, armor_predict_duration_,
                    buff_predict_duration_);
            }};
            return;
        }

        auto gimbal_pose =
            fast_tf::lookup_transform<rmcs_description::OdomImu, rmcs_description::PitchLink>(*tf_);
        transformer::SetRotation<GimbalGyro, GimbalLink>(gimbal_pose);

        auto target = target_;
        if (!target_) {
            return;
        }

        using namespace std::chrono_literals;
        auto diff = std::chrono::steady_clock::now() - timestamp_;
        if (diff > std::chrono::milliseconds(
                armor_predict_duration_ < buff_predict_duration_ ? armor_predict_duration_
                                                                 : buff_predict_duration_)) {
            *control_direction_ = Eigen::Vector3d::Zero();
            return;
        }

        double fly_time = 0;
        for (int i = 5; i-- > 0;) {
            auto pos = target->Predict(
                static_cast<std::chrono::duration<double>>(diff).count() + fly_time + 0.05);
            auto aiming_direction = *trajectory_.GetShotVector(pos, 27.0, fly_time);
            auto delta_yaw = Eigen::AngleAxisd{yaw_error, gimbal_pose * Eigen::Vector3d::UnitZ()};
            auto delta_pitch =
                Eigen::AngleAxisd{pitch_error, gimbal_pose * Eigen::Vector3d::UnitY()};
            aiming_direction = delta_pitch * (delta_yaw * (aiming_direction));
            if (i == 0) {
                *control_direction_ = aiming_direction;
            }
        }
    }

private:
    InputInterface<size_t> update_count_;
    InputInterface<rmcs_core::msgs::RoboticColor> color_;
    InputInterface<uint8_t> robot_id_;
    InputInterface<rmcs_description::Tf> tf_;
    InputInterface<bool> buff_mode_;

    int64_t exposure_time_;
    int64_t armor_predict_duration_;
    int64_t buff_predict_duration_;

    double yaw_error;
    double pitch_error;

    std::unique_ptr<GimbalInfantry> gimbal_;
    std::thread gimbal_thread_;

    TargetInterface* target_;
    std::chrono::steady_clock::time_point timestamp_;
    Trajectory_V1 trajectory_{};

    OutputInterface<Eigen::Vector3d> control_direction_;
};

}; // namespace ugas

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ugas::Component, rmcs_executor::Component)
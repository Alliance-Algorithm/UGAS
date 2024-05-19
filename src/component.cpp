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
        register_output(
            "/gimbal/auto_aim/control_direction", control_direction_, Eigen::Vector3d::Zero());

        gimbal_ = std::make_unique<GimbalInfantry>();
    }

    ~Component() {
        if (gimbal_thread_.joinable())
            gimbal_thread_.join();
    }

    void update() override {
        if (*update_count_ == 0) {
            gimbal_thread_ = std::thread{
                [this]() { gimbal_->Always(target_, timestamp_, *color_, *robot_id_); }};
            return;
        }

        auto gimbal_pose =
            fast_tf::lookup_transform<rmcs_description::OdomImu, rmcs_description::PitchLink>(*tf_);
        transformer::SetRotation<GimbalGyro, GimbalLink>(gimbal_pose);

        auto target = target_;
        if (!target_)
            return;

        using namespace std::chrono_literals;
        auto diff = std::chrono::steady_clock::now() - timestamp_;
        if (diff > 500ms) {
            *control_direction_ = Eigen::Vector3d::Zero();
            return;
        }

        double fly_time = 0;
        for (int i = 5; i-- > 0;) {
            auto pos = target->Predict(
                static_cast<std::chrono::duration<double>>(diff).count() + fly_time + 0.05);
            auto aiming_direction = *trajectory_.GetShotVector(pos, 27.0, fly_time);
            auto delta_yaw   = Eigen::AngleAxisd{0.005, gimbal_pose * Eigen::Vector3d::UnitZ()};
            auto delta_pitch = Eigen::AngleAxisd{0.050, gimbal_pose * Eigen::Vector3d::UnitY()};
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
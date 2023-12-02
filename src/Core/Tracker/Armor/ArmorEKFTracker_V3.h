#pragma once

/*
Creation Date: 2023/07/24
Latest Update: 2023/07/24
Developer(s): 22-Qzh
Reference(s): ChenJun armor_tracker
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 基于EKF的目标追踪器 第二版
*/

#include <eigen3/Eigen/Dense>

#include "Core/Tracker/TrackerStruct.h"
#include "Core/Tracker/Armor/EKF.h"
#include "Util/ROS/TfBroadcast.h"

class ArmorEKFTracker {
    // 整车追踪器
    struct TrackerUnit {
        TrackerUnit(const ArmorPlate3d& armor, const std::chrono::steady_clock::time_point& timestamp) : last_update(timestamp) {
            double& r = r_list[0];
            double yaw = GetArmorYaw(armor);
            double xc = armor.position->x() + r * cos(yaw);
            double yc = armor.position->y() + r * sin(yaw);
            const double& za = armor.position->z();
            // xc  v_xc  yc  v_yc  za  v_za  yaw  v_yaw  r
            ekf.x_ << xc, 0, yc, 0, za, 0, yaw, 0, r;

#if ENABLE_ROS
            ros_marker_id = ros_marker_id_global_;
            ros_marker_id_global_ += armor_count;
            color_r = (float)((double)rand() / ((double)RAND_MAX + 1));
            color_g = (float)((double)rand() / ((double)RAND_MAX + 1));
            color_b = (float)((double)rand() / ((double)RAND_MAX + 1));
#endif
        }

        void Predict(double dt) {
            ekf.Predict(dt);
            tracked_duration += dt;
            for (bool &updated : armor_newly_updated)
                updated = false;
        }

        void Update(const ArmorPlate3d& armor, const std::chrono::steady_clock::time_point& timestamp) {
            double &v_za = ekf.x_(5), & model_yaw = ekf.x_(6), & r = ekf.x_(8);
            double yaw = GetArmorYaw(armor);

            constexpr double legal_range = parameters::Pi / 4;
            constexpr double step = 2 * parameters::Pi / armor_count;

            double shift = 0;

            size_t i;
            for (i = 0; i < armor_count; ++i) {
                //std::cout << yaw << ' ';
                double diff = GetMinimumAngleDiff(yaw, model_yaw + shift);
                if (-legal_range < diff && diff < legal_range) {
                    yaw = model_yaw + shift + diff;
                    break;
                }
                else shift += step;
            }

            if (i < 4) {
                model_yaw += shift;
                r = r_list[i];

                Eigen::Vector4d measurement = { armor.position->x(), armor.position->y(), armor.position->z(), yaw };
                ekf.Update(measurement);
                last_update = timestamp;

                v_za = 0;
                if (r > -0.12)
                    r = -0.12;
                else if (r < -0.4)
                    r = -0.4;
                if (i == 0 || i == 2)
                    r_list[0] = r_list[2] = r;
                else if (i == 1 || i == 3)
                    r_list[1] = r_list[3] = r;
                model_yaw -= shift;

                tracked_times += 1;
                if (tracked_duration > 0.5)
                    tracking_density = tracked_times / tracked_duration;

                armor_newly_updated[i] = true;
            }
        }

        [[nodiscard]] static std::tuple<Eigen::Vector3d, double> GetArmorState(const Eigen::VectorXd& x, size_t index) {
            const double& xc = x(0), & yc = x(2), & za = x(4), & r = x(8);
            constexpr double yaw_step = 2 * parameters::Pi / armor_count;
            double yaw = x(6) + yaw_step * (double)index;

            double xa = xc - r * cos(yaw);
            double ya = yc - r * sin(yaw);

            return {Eigen::Vector3d(xa, ya, za), yaw};
        }

        [[nodiscard]] std::tuple<Eigen::Vector3d, double> GetArmorState(size_t index) const {
            Eigen::VectorXd x = ekf.x_;
            x(8) = r_list[index];
            return GetArmorState(x, index);
        }

        EKF ekf;
        static constexpr size_t armor_count = 4;
        std::chrono::steady_clock::time_point last_update;
        double r_list[armor_count]{-0.26, -0.26, -0.26, -0.26};
        bool armor_newly_updated[armor_count]{false, false, false, false};

        double tracked_duration = 0, tracked_times = 0, tracking_density = 0;

#if ENABLE_ROS
        int ros_marker_id;
        float color_r, color_g, color_b;
#endif
    };

    class Target : public TargetInterface {
    public:
        explicit Target(const TrackerUnit& tracker) : tracker_(tracker) { }

        [[nodiscard]] GimbalGyro::Position Predict(double sec) const override {
            // xc  v_xc  yc  v_yc  za  v_za  yaw  v_yaw  r
            Eigen::VectorXd x = tracker_.ekf.PredictConst(sec);
            const double& xc = x(0), & yc = x(2), & za = x(4), & v_yaw = x(7);
            double& model_yaw = x(6);
            double camera_yaw = std::atan2(-yc, -xc);
            if (fabs(v_yaw) < 2.0) {
                double shift = 0;
                constexpr double legal_range = parameters::Pi / 4;
                constexpr double step = 2 * parameters::Pi / TrackerUnit::armor_count;
                size_t i;
                for (i = 0; i < TrackerUnit::armor_count; ++i) {
                    double diff = GetMinimumAngleDiff(camera_yaw, model_yaw + shift);
                    if (-legal_range < diff && diff < legal_range)
                        break;
                    else shift += step;
                }
                if (i < TrackerUnit::armor_count) {
                    double r = tracker_.r_list[i];
                    double yaw = model_yaw + shift;
                    auto pos = Eigen::Vector3d{xc - r * cos(yaw), yc - r * sin(yaw), za};
                    return GimbalGyro::Position(pos);
                }
            }
            //return GimbalGyro::Position(0, 0, 0);
            model_yaw = camera_yaw;
            double r = (tracker_.r_list[0] + tracker_.r_list[1]) / 2;
            //auto [pos, yaw] = TrackerUnit::GetArmorState(x, 0);
            auto pos = Eigen::Vector3d{xc - r * cos(camera_yaw), yc - r * sin(camera_yaw), za};

            return GimbalGyro::Position(pos);
        }

    private:
        const TrackerUnit& tracker_;
    };

public:
    ArmorEKFTracker() {
        tracker_map_[ArmorID::Hero] = {};
        tracker_map_[ArmorID::Engineer] = {};
        tracker_map_[ArmorID::InfantryIII] = {};
        tracker_map_[ArmorID::InfantryIV] = {};
        tracker_map_[ArmorID::InfantryV] = {};
        tracker_map_[ArmorID::Sentry] = {};
        tracker_map_[ArmorID::Outpost] = {};
    };

    std::unique_ptr<TargetInterface> Update(const std::vector<ArmorPlate3d>& armors, std::chrono::steady_clock::time_point timestamp) {
        // dt: interval between adjacent updates by seconds.
        double dt = std::chrono::duration<double>(timestamp - last_update_).count();
        last_update_ = timestamp;

        for (auto &[armor_id, tracker_array]: tracker_map_) {
            for (auto iter = tracker_array.begin(); iter != tracker_array.end();) {
                auto &tracker = *iter;
                if (timestamp - tracker.last_update > std::chrono::milliseconds(1000))
                    iter = tracker_array.erase(iter);
                else {
                    tracker.Predict(dt);
                    ++iter;
                }
            }
        }

        for (const auto& armor : armors) {
            auto iter = tracker_map_.find(armor.id);
            if (iter != tracker_map_.end()) {
                auto& tracker_array = iter->second;

                if (tracker_array.empty()) {
                    tracker_array.emplace_back(armor, timestamp);
                }
                else {
                    auto& tracker = tracker_array[0];
                    tracker.Update(armor, timestamp);
                }
            }
        }
        //return;


#if ENABLE_ROS
        // 发送装甲板可视化信息
        if (auto node = ros_util::node_.lock()) {
            visualization_msgs::msg::MarkerArray marker_array;
            for (auto& [armor_id, tracker_array]: tracker_map_) {
                for (auto& tracker : tracker_array) {
                    if (tracker.tracking_density < 40.0) continue;
                    //std::cout << (int)armor_id << ' ' << tracker.ekf.x_(7) << ' ';
                    //std::cout << tracker.ekf.x_[7] << '\n';
                    for (size_t i = 0; i < TrackerUnit::armor_count; ++i) {
                        visualization_msgs::msg::Marker marker1;
                        marker1.header.frame_id = "gimbal_gyro";
                        marker1.ns = "armor_plate_array";
                        marker1.id = tracker.ros_marker_id + (int)i;
                        marker1.type = visualization_msgs::msg::Marker::CUBE;
                        marker1.action = visualization_msgs::msg::Marker::MODIFY;
                        marker1.scale.x = 0.02;
                        marker1.scale.y = 0.135;
                        marker1.scale.z = 0.125;
                        marker1.color.r = tracker.color_r;
                        marker1.color.g = tracker.color_g;
                        marker1.color.b = tracker.color_b;
                        marker1.color.a = 0.5;
                        marker1.lifetime = rclcpp::Duration::from_seconds(0.1);
                        marker1.header.stamp = node->now();
                        auto [pos, yaw] = tracker.GetArmorState(i);
                        marker1.pose.position.x = pos.x();
                        marker1.pose.position.y = pos.y();
                        marker1.pose.position.z = pos.z();
                        auto rotation = Eigen::Quaterniond{Eigen::AngleAxisd{yaw, Eigen::Vector3d::UnitZ()}};
                        marker1.pose.orientation.w = rotation.w();
                        marker1.pose.orientation.x = rotation.x();
                        marker1.pose.orientation.y = rotation.y();
                        marker1.pose.orientation.z = rotation.z();
                        marker_array.markers.push_back(std::move(marker1));
                    }

                }
            }
            //std::cout << '\n';

            static int debug_index = 0;
            for (auto& armor : armors) {
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "gimbal_gyro";
                marker.ns = "armor_plate_array";
                marker.id = 1000000 + debug_index++;
                marker.type = visualization_msgs::msg::Marker::CUBE;
                marker.action = visualization_msgs::msg::Marker::MODIFY;
                marker.scale.x = 0.02;
                marker.scale.y = 0.135;
                marker.scale.z = 0.125;
                marker.color.r = 1.0;
                marker.color.g = 1.0;
                marker.color.b = 1.0;
                marker.color.a = 0.5;
                marker.lifetime = rclcpp::Duration::from_seconds(0.1);
                marker.header.stamp = node->now();
                marker.pose.position.x = armor.position->x();
                marker.pose.position.y = armor.position->y();
                marker.pose.position.z = armor.position->z();
                auto rotation = Eigen::Quaterniond{Eigen::AngleAxisd{GetArmorYaw(armor), Eigen::Vector3d::UnitZ()}};
                marker.pose.orientation.w = rotation.w();
                marker.pose.orientation.x = rotation.x();
                marker.pose.orientation.y = rotation.y();
                marker.pose.orientation.z = rotation.z();
                marker_array.markers.push_back(std::move(marker));
            }
            node->marker_pub_->publish(marker_array);
        }
#endif
        TrackerUnit* selected_tracker = nullptr;
        int selected_level = 0; double minimum_angle;
        for (auto &[armor_id, tracker_array]: tracker_map_) {
            for (auto& tracker : tracker_array) {
                int level = 0;
                if (tracker.tracking_density > 100) level = 2;
                else if (tracker.tracking_density > 40) level = 1;

                auto center = *static_cast<MuzzleLink::Position>(
                        GimbalGyro::Position(tracker.ekf.x_(0), tracker.ekf.x_(2), tracker.ekf.x_(4)));
                double angle = std::acos(center.dot(Eigen::Vector3d{1, 0, 0}) / center.norm());

                if (level > selected_level) {
                    selected_level = level;
                    minimum_angle = angle;
                    selected_tracker = &tracker;
                }
                else if(level > 0 && level == selected_level && angle < minimum_angle) {
                    minimum_angle = angle;
                    selected_tracker = &tracker;
                }
            }
        }
        if (selected_tracker) {
            return std::make_unique<Target>(*selected_tracker);
        }
        else
            return nullptr;
    }

private:
    static double GetArmorYaw(const ArmorPlate3d& armor) {
        Eigen::Vector3d normal = (*armor.rotation) * Eigen::Vector3d{1, 0, 0};
        return atan2(normal.y(), normal.x());
    }

    static double GetMinimumAngleDiff(double a, double b) {
        double diff = std::fmod(a - b, 2 * parameters::Pi);
        if (diff < -parameters::Pi) diff += 2 * parameters::Pi;
        else if (diff > parameters::Pi) diff -= 2 * parameters::Pi;
        return diff;
    }

    // Generate continuous yaw (-pi~pi -> -inf~inf)
    static double GetContinuousYaw(const ArmorPlate3d& armor, double last_yaw) {
        double yaw = GetArmorYaw(armor);
        double diff = GetMinimumAngleDiff(yaw, last_yaw);
        return last_yaw + diff;
    }

    //std::list<TrackerUnit> tracker_array_;
    std::map<ArmorID, std::vector<TrackerUnit>> tracker_map_;
    std::chrono::steady_clock::time_point last_update_;

#if ENABLE_ROS
    static inline int ros_marker_id_global_ = 0;
#endif
};
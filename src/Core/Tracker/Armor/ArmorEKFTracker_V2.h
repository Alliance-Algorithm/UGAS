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

#include "Core/Tracker/Armor/EKF.h"
#include "Util/ROS/TfBroadcast.h"

class ArmorEKFTracker {
public:
    ArmorEKFTracker() = default;

    void Update(const std::vector<ArmorPlate3d>& armors, std::chrono::steady_clock::time_point timestamp) {
        auto **armor_match_tracker_array = new TrackerUnit*[armors.size()];
        for (size_t i = 0; i < armors.size(); ++i)
            armor_match_tracker_array[i] = nullptr;

        // 更新现有追踪器
        for (auto it=tracker_array_.begin(); it!=tracker_array_.end();) {
            auto& tracker = *it;
            //  若追踪器超过一定时间未更新，则销毁追踪器
            if (timestamp - tracker.last_update > std::chrono::milliseconds(100)) {
                it = tracker_array_.erase(it);
                continue;
            }
            else ++it;

            // 追踪器尝试匹配距离最近的装甲板
            bool tracker_matched_armor = false;
            size_t tracker_match_armor_index;
            double matched_distance;
            for (size_t i = 0; i < armors.size(); ++i) {
                const auto& armor = armors[i];
                if (armor.id == tracker.id) {
                    double distance = (tracker.position - *armor.position).norm();
                    if (distance < 0.1 && (!tracker_matched_armor || distance < matched_distance)) {
                        tracker_matched_armor = true;
                        tracker_match_armor_index = i;
                    }
                }
            }

            // 追踪器若匹配到装甲板，更新追踪器
            // 若出现多个追踪器匹配到同个装甲板，只保留离此装甲板最近的追踪器，销毁其余追踪器
            if (tracker_matched_armor) {
                if (armor_match_tracker_array[tracker_match_armor_index] != nullptr) {
                    double exist_match_distance = (tracker.position - armor_match_tracker_array[tracker_match_armor_index]->position).norm();
                    double distance = (tracker.position - *armor.position).norm();
                }
                armor_match_tracker_array[tracker_match_armor_index] = &tracker;
                const auto& matched_armor = armors[tracker_match_armor_index];
                tracker.Update(matched_armor, timestamp);
            }
        }

        // 对未匹配成功的装甲板，创建新追踪器
        for (size_t i = 0; i < armors.size(); ++i) {
            if (!is_armor_matched[i]) {
                tracker_array_.emplace_back(armors[i], timestamp);
            }
        }

        delete[] matched_armor;



#if ENABLE_ROS
        // 发送装甲板可视化信息
        if (auto node = ros_util::node_.lock()) {
            visualization_msgs::msg::MarkerArray marker_array;
            for (const auto& tracker : tracker_array_) {
                visualization_msgs::msg::Marker marker1;
                marker1.header.frame_id = "gimbal_gyro";
                marker1.ns = "armor_plate_array";
                marker1.id = tracker.ros_marker_id;
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
                marker1.pose.position.x = tracker.position.x();
                marker1.pose.position.y = tracker.position.y();
                marker1.pose.position.z = tracker.position.z();
                //std::cout << marker.pose.position.x << ' ' << marker.pose.position.y << ' ' << marker.pose.position.z << '\n';
                auto rotation = Eigen::Quaterniond{Eigen::AngleAxisd{tracker.yaw, Eigen::Vector3d::UnitZ()}};
                marker1.pose.orientation.w = rotation.w();
                marker1.pose.orientation.x = rotation.x();
                marker1.pose.orientation.y = rotation.y();
                marker1.pose.orientation.z = rotation.z();
                marker_array.markers.push_back(std::move(marker1));

                visualization_msgs::msg::Marker marker2;
                marker2.header.frame_id = "gimbal_gyro";
                marker2.ns = "armor_plate_array";
                marker2.id = tracker.ros_marker_id + 1000000;
                marker2.type = visualization_msgs::msg::Marker::SPHERE;
                marker2.action = visualization_msgs::msg::Marker::MODIFY;
                marker2.scale.x = 0.05;
                marker2.scale.y = 0.05;
                marker2.scale.z = 0.05;
                marker2.color.r = tracker.color_r;
                marker2.color.g = tracker.color_g;
                marker2.color.b = tracker.color_b;
                marker2.color.a = 0.5;
                marker2.lifetime = rclcpp::Duration::from_seconds(0.1);
                marker2.header.stamp = node->now();
                marker2.pose.position.x = tracker.ekf.x_[0];
                marker2.pose.position.y = tracker.ekf.x_[2];
                marker2.pose.position.z = tracker.ekf.x_[4];
                marker_array.markers.push_back(std::move(marker2));
            }
            node->marker_pub_->publish(marker_array);
        }
#endif
    }

private:
    static double GetContinuousYaw(const ArmorPlate3d& armor3d, double last_yaw) {
        Eigen::Vector3d normal = (*armor3d.rotation) * Eigen::Vector3d{1, 0, 0};
        double yaw = atan2(normal.y(), normal.x());

        // Generate continuous yaw (-pi~pi -> -inf~inf)
        double diff = std::fmod(yaw - last_yaw, 2 * MathConsts::Pi);
        if (diff < -MathConsts::Pi) diff += 2 * MathConsts::Pi;
        else if (diff > MathConsts::Pi) diff -= 2 * MathConsts::Pi;

        return last_yaw + diff;
    }

    // 用于追踪单个装甲板
    struct TrackerUnit {
        TrackerUnit(const ArmorPlate3d& armor, const std::chrono::steady_clock::time_point& timestamp) :
            id(armor.id), position(*armor.position), yaw(GetContinuousYaw(armor, 0)),
            last_update(timestamp) {

            double r = -0.26;
            double xc = position.x() + r * cos(yaw);
            double yc = position.y() + r * sin(yaw);
            const double& za = position.z();
            // xc  v_xc  yc  v_yc  za  v_za  yaw  v_yaw  r
            ekf.x_ << xc, 0, yc, 0, za, 0, yaw, 0, r;

#if ENABLE_ROS
            ros_marker_id = ros_marker_id_global_++;
            color_r = (float)((double)rand() / ((double)RAND_MAX + 1));
            color_g = (float)((double)rand() / ((double)RAND_MAX + 1));
            color_b = (float)((double)rand() / ((double)RAND_MAX + 1));
#endif
        }

        void Update(const ArmorPlate3d& armor, const std::chrono::steady_clock::time_point& timestamp) {
            // Interval between adjacent updates by seconds.
            double dt = std::chrono::duration<double>(timestamp - last_update).count();
            ekf.predict(dt);

            position = *armor.position;
            yaw = GetContinuousYaw(armor, yaw);
            last_update = timestamp;

            Eigen::Vector4d measurement = { position.x(), position.y(), position.z(), yaw };
            ekf.update(measurement);
            //ekf._x(1) = ekf._x(3) = ekf._x(5) = 0;

            if (ekf.x_(8) > -0.12)
                ekf.x_(8) = -0.12;
            else if (ekf.x_(8) < -0.4)
                ekf.x_(8) = -0.4;
        }

        ArmorID id;
        Eigen::Vector3d position;
        double yaw;
        EKF ekf;
        std::chrono::steady_clock::time_point last_update;
#if ENABLE_ROS
        int ros_marker_id;
        float color_r, color_g, color_b;
#endif
    };

    std::list<TrackerUnit> tracker_array_;
#if ENABLE_ROS
    static inline int ros_marker_id_global_ = 0;
#endif
};
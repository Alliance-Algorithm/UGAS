#pragma once
/*
Creation Date: 2023/06/23
Latest Update: 2023/06/23
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 进行Tf2广播
*/

#if ENABLE_ROS

#include <eigen3/Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>


#include "Util/ROS/Node.h"

#endif // ENABLE_ROS

namespace ros_util {
    inline void TfBroadcast(const char* header, const char* child, const Eigen::Vector3d& translation, const Eigen::Quaterniond& rotation) {
#if ENABLE_ROS
        if (auto node = node_.lock()) {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = node->get_clock()->now();
            t.header.frame_id = header;
            t.child_frame_id = child;

            t.transform.translation.x = translation.x();
            t.transform.translation.y = translation.y();
            t.transform.translation.z = translation.z();

            t.transform.rotation.w = rotation.w();
            t.transform.rotation.x = rotation.x();
            t.transform.rotation.y = rotation.y();
            t.transform.rotation.z = rotation.z();

            node->tf_broadcaster_.sendTransform(t);
        }
#endif // ENABLE_ROS
    }

    inline void TfBroadcast(const char* header, const char* child, const Eigen::Isometry3d& transform) {
#if ENABLE_ROS
        if (auto node = node_.lock()) {
            Eigen::Vector3d translation{ transform.translation() };
            Eigen::Quaterniond rotation{ transform.linear() };

            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = node->get_clock()->now();
            t.header.frame_id = header;
            t.child_frame_id = child;

            t.transform.translation.x = translation.x();
            t.transform.translation.y = translation.y();
            t.transform.translation.z = translation.z();

            t.transform.rotation.w = rotation.w();
            t.transform.rotation.x = rotation.x();
            t.transform.rotation.y = rotation.y();
            t.transform.rotation.z = rotation.z();

            node->tf_broadcaster_.sendTransform(t);
        }
#endif // ENABLE_ROS
    }

    template <typename Header, typename Child>
    inline void TfBroadcast() {
#if ENABLE_ROS
        Eigen::Isometry3d transform = transformer::CalculateTransform<Header, Child>();
        ros_util::TfBroadcast(Header::name, Child::name, transform);
#endif // ENABLE_ROS
    }

    inline void PointBroadcast(const char* header, const Eigen::Vector3d& coord) {
#if ENABLE_ROS
        if (auto node = node_.lock()) {
            /*visualization_msgs::msg::Marker aiming_point_;
            aiming_point_.header.frame_id = header;
            aiming_point_.ns = "aiming_point";
            aiming_point_.type = visualization_msgs::msg::Marker::SPHERE;
            aiming_point_.action = visualization_msgs::msg::Marker::ADD;
            aiming_point_.scale.x = aiming_point_.scale.y = aiming_point_.scale.z = 0.12;
            aiming_point_.color.r = 1.0;
            aiming_point_.color.g = 1.0;
            aiming_point_.color.b = 1.0;
            aiming_point_.color.a = 1.0;
            aiming_point_.lifetime = rclcpp::Duration::from_seconds(0.1);
            aiming_point_.header.stamp = node->now();
            aiming_point_.pose.position.x = coord.x();
            aiming_point_.pose.position.y = coord.y();
            aiming_point_.pose.position.z = coord.z();
            node->marker_pub_->publish(aiming_point_);*/
            geometry_msgs::msg::PointStamped p;
            p.header.stamp = node->get_clock()->now();
            p.header.frame_id = header;
            p.point.x = coord.x();
            p.point.y = coord.y();
            p.point.z = coord.z();
            node->point_pub_->publish(p);
        }
#endif
    }

    template <typename PositionType>
    inline void PointBroadcast(PositionType position) {
#if ENABLE_ROS
        PointBroadcast(PositionType::Node::name, *position);
#endif
    }

    inline void ArmorPlateBroadcast(const std::vector<ArmorPlate3d>& armors, const std::vector<int>& armor_ids) {
#if ENABLE_ROS
        if (auto node = node_.lock()) {
            visualization_msgs::msg::MarkerArray marker_array;
            size_t i = 0;
            for (const auto& armor : armors) {
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "gimbal_gyro";
                marker.ns = "armor_plate_array";
                marker.id = armor_ids[i++];
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
                marker.pose.position.x = armor.position->x() / 1000;
                marker.pose.position.y = armor.position->y() / 1000;
                marker.pose.position.z = armor.position->z() / 1000;
                marker.pose.orientation.w = armor.rotation->w();
                marker.pose.orientation.x = armor.rotation->x();
                marker.pose.orientation.y = armor.rotation->y();
                marker.pose.orientation.z = armor.rotation->z();
                marker_array.markers.push_back(std::move(marker));
            }
            node->marker_pub_->publish(marker_array);
        }
#endif
    }
}
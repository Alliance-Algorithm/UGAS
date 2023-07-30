#pragma once
/*
Creation Date: 2023/06/23
Latest Update: 2023/06/23
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 在一个独立线程中运行ROS2 Node
*/

#include "config.h"

#if ENABLE_ROS

#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

namespace ros_util {
    class Node : public rclcpp::Node {
    public:
        Node() : rclcpp::Node("main"), tf_broadcaster_(*this) {
            point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/test_point", 10);
            marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/armor_plate_array", 10);
        }
        ~Node() override = default;

        tf2_ros::TransformBroadcaster tf_broadcaster_;

        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    };

    inline std::thread node_thread_;
    inline std::weak_ptr<Node> node_;

    inline void init(int argc, char* argv[]) {
        node_thread_ = std::thread([argc, argv](){
            rclcpp::init(argc, argv);
            auto node = std::make_shared<Node>();
            node_ = node;
            rclcpp::spin(node);
            rclcpp::shutdown();
        });
        node_thread_.detach();
    }
}

#else // ENABLE_ROS

namespace ros_util {
    void init(int argc, char* argv[]) { }
}

#endif // ENABLE_ROS
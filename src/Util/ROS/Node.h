#pragma once
/*
Creation Date: 2023/06/23
Latest Update: 2023/06/23
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 在一个独立线程中运行ROS2 Node
*/

#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace ros_util {
    class Node : public rclcpp::Node {
    public:
        Node() : rclcpp::Node("main") {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            thread_ = std::thread(&Node::thread_main, this);
        }

        ~Node() override {
            if (thread_.joinable()) {
                thread_.join();
            }
        }

    private:
        void thread_main() {
            double yaw = 0;
            while (rclcpp::ok()) {
                geometry_msgs::msg::TransformStamped t;
                t.header.stamp = this->get_clock()->now();
                t.header.frame_id = "world";
                t.child_frame_id = "abc";

                t.transform.translation.x = 10.0;
                t.transform.translation.y = 10.0;
                t.transform.translation.z = 0.0;

                tf2::Quaternion q;
                q.setRPY(0, 0, yaw);
                t.transform.rotation.x = q.x();
                t.transform.rotation.y = q.y();
                t.transform.rotation.z = q.z();
                t.transform.rotation.w = q.w();

                tf_broadcaster_->sendTransform(t);

                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                yaw += 0.001;
            }
        }

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        std::thread thread_;
    };

    std::thread node_thread_;
    std::weak_ptr<Node> node_;

    void init(int argc, char* argv[]) {
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
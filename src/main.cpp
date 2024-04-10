
#include <ctime>
#include <thread>

#include <eigen3/Eigen/Dense>
#include <hikcamera/image_capturer.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

#include "core/ballistic_solver/ballistic_solver.hpp"
#include "core/detector/armor/armor_detector.hpp"
#include "util/fps_counter.h"
#include "util/timer.hpp"

class CameraNode : public rclcpp::Node {
public:
    CameraNode(const std::string& node_name)
        : Node(node_name)
        , tf_buffer_(get_clock())
        , tf_listener_(tf_buffer_)
        , thread_(&CameraNode::thread_main, this) {
        aiming_direction_publisher_ =
            create_publisher<geometry_msgs::msg::Vector3>("/gimbal/auto_aim", rclcpp::QoS(1));
        marker_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
            "/gimbal/auto_aim_marker", rclcpp::QoS(1));
    }

private:
    void thread_main() {
        FpsCounter fps_counter;
        Timer timer;

        hikcamera::ImageCapturer::CameraProfile camera_profile;
        {
            using namespace std::chrono_literals;
            camera_profile.exposure_time = 3ms;
            camera_profile.gain          = 16.9807;
            camera_profile.invert_image  = true;
        }
        hikcamera::ImageCapturer image_capturer(camera_profile);
        ArmorDetector armor_detector;
        BallisticSolver ballistic_solver;

        while (rclcpp::ok()) {
            if (fps_counter.count())
                RCLCPP_INFO(this->get_logger(), "fps: %d ", fps_counter.get_fps());

            auto image  = image_capturer.read();
            auto armors = armor_detector.detect(image, ArmorDetector::ArmorColor::BLUE);

            geometry_msgs::msg::Vector3 msg;

            if (armors.empty()) {
                if (timer.running()) {
                    if (timer.elapsed() >= 500) {
                        timer.reset();

                        msg.x = 0;
                        msg.y = 0;
                        msg.z = 0;
                        aiming_direction_publisher_->publish(msg);
                    }
                } else {
                    timer.start();
                }
                continue;
            }

            geometry_msgs::msg::TransformStamped camera_link_to_odom, odom_to_muzzle_link;
            try {
                camera_link_to_odom =
                    tf_buffer_.lookupTransform("odom", "camera_link", tf2::TimePointZero);
                odom_to_muzzle_link =
                    tf_buffer_.lookupTransform("odom", "muzzle_link", tf2::TimePointZero);
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
                continue;
            }
            auto gimbal_pose = Eigen::Quaterniond{
                camera_link_to_odom.transform.rotation.w, camera_link_to_odom.transform.rotation.x,
                camera_link_to_odom.transform.rotation.y, camera_link_to_odom.transform.rotation.z};

            Eigen::Vector3d target =
                Eigen::Translation3d{
                    camera_link_to_odom.transform.translation.x,
                    camera_link_to_odom.transform.translation.y,
                    camera_link_to_odom.transform.translation.z}
                * (gimbal_pose * armors[0].position);

            visualization_msgs::msg::Marker aiming_point_;
            aiming_point_.header.frame_id = "odom";
            aiming_point_.type            = visualization_msgs::msg::Marker::SPHERE;
            aiming_point_.action          = visualization_msgs::msg::Marker::ADD;
            aiming_point_.scale.x = aiming_point_.scale.y = aiming_point_.scale.z = 0.05;
            aiming_point_.color.r                                                 = 1.0;
            aiming_point_.color.g                                                 = 0.0;
            aiming_point_.color.b                                                 = 0.0;
            aiming_point_.color.a                                                 = 1.0;
            aiming_point_.lifetime        = rclcpp::Duration::from_seconds(0.1);
            aiming_point_.header.stamp    = now();
            aiming_point_.pose.position.x = target.x();
            aiming_point_.pose.position.y = target.y();
            aiming_point_.pose.position.z = target.z();
            marker_publisher_->publish(aiming_point_);

            Eigen::Vector3d muzzle = {
                odom_to_muzzle_link.transform.translation.x,
                odom_to_muzzle_link.transform.translation.y,
                odom_to_muzzle_link.transform.translation.z};

            auto aiming_direction = ballistic_solver.solve(target, muzzle, 28.0);

            auto delta_yaw   = Eigen::AngleAxisd{-0.008, gimbal_pose * Eigen::Vector3d::UnitZ()};
            auto delta_pitch = Eigen::AngleAxisd{0.000, gimbal_pose * Eigen::Vector3d::UnitY()};
            aiming_direction = (delta_pitch * (delta_yaw * aiming_direction)).eval();

            msg.x = aiming_direction.x();
            msg.y = aiming_direction.y();
            msg.z = aiming_direction.z();
            aiming_direction_publisher_->publish(msg);
        }
    }

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr aiming_direction_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;

    std::thread thread_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;

    auto node = std::make_shared<CameraNode>("main_camera");
    executor.add_node(node);

    executor.spin();
    rclcpp::shutdown();
    return 0;
}
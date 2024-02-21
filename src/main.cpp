#include <opencv2/highgui.hpp>
#include <thread>

#include <hikcamera/image_capturer.hpp>
#include <rclcpp/rclcpp.hpp>

#include "core/detector/armor/armor_detector.hpp"

class CameraNode : public rclcpp::Node {
public:
    CameraNode(const std::string& node_name)
        : Node(node_name)
        , thread_(&CameraNode::thread_main, this) {}

private:
    void thread_main() {
        hikcamera::ImageCapturer image_capturer;

        ArmorDetector armor_detector;

        while (rclcpp::ok()) {
            auto image = image_capturer.read();
            auto armors = armor_detector.detect(image, ArmorDetector::ArmorColor::BLUE);
            cv::imshow("image", image);
            cv::waitKey(1);
        }
    }

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
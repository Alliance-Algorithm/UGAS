#include "GimbalInfantry.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <hikcamera/image_capturer.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_core/msgs.hpp>
#include <thread>

#include "Core/ImgCapture/Common/CVVideoCapture.h"
#include "Util/Parameter/Parameters.h"
#include "config.h"
// #include "Core/ImgCapture/Common/HikCameraCapture.h"
#include "Core/Identifier/Armor/ArmorIdentifier_V3.h"
#include "Core/Identifier/Buff/BuffIdentifier_V1.h"
#include "Core/Identifier/Number/NumberIdentifier_V1.h"
#include "Core/ImgCapture/Common/ImageFolderCapture.h"
#include "Core/PnPSolver/Armor/ArmorPnPSolver.h"
#include "Core/PnPSolver/Buff/BuffPnPSolver.h"
#include "Core/Predictor/Armor/SimplePredictor.h"
#include "Core/Pretreator/Armor/ArmorPretreator_V2.h"
#include "Core/Tracker/Armor/ArmorEKFTracker_V3.h"
#include "Core/Tracker/Buff/BuffTracker.h"
#include "Core/Trajectory/Common/Trajectory_V1.h"
// #include "Control/Serial/CBoardInfantry.h"
#include "Control/Serial/CBoardInfantryAsync.h"
// #include "Control/Serial/VirtualCBoard.h"
#include "Util/Debug/DebugCanvas.h"
#include "Util/FPSCounter/FPSCounter.h"
#include "Util/ROS/TfBroadcast.h"
#include "Util/Recorder/PNGRecorder.h"

inline const rclcpp::QoS kCoreQoS = rclcpp::QoS(1).best_effort().durability_volatile();

// class TestSender {
// public:
//     TestSender()
//         : thread_(&TestSender::thread_main, this) {
//         if (auto node = ros_util::node_.lock()) {
//             aiming_direction_publisher_ =
//                 node->create_publisher<geometry_msgs::msg::Vector3>("/gimbal/auto_aim",
//                 kCoreQoS);

//             gimbal_pose_subscription_ =
//             node->create_subscription<geometry_msgs::msg::Quaternion>(
//                 "/gimbal/pose_imu", kCoreQoS, [](geometry_msgs::msg::Quaternion::UniquePtr msg) {
//                     transformer::SetRotation<GimbalGyro, GimbalLink>(
//                         Eigen::Quaterniond{msg->w, msg->x, msg->y, msg->z});
//                     // ros_util::TfBroadcast<GimbalGyro, CameraLink>();
//                     // ros_util::TfBroadcast<GimbalGyro, MuzzleLink>();
//                     // ros_util::TfBroadcast<GimbalGyro, TransmitterLink>();
//                 });
//         } else
//             throw std::runtime_error{"ohhhh"};
//     }

//     ~TestSender() { thread_.join(); }

//     void update(
//         std::unique_ptr<TargetInterface> target, std::chrono::steady_clock::time_point timestamp)
//         { auto p          = target.release(); auto previous_p =
//         target_.load(std::memory_order_acquire); target_.store(p, std::memory_order_relaxed);
//         timestamp_.store(timestamp, std::memory_order_release);
//         // delete previous_p;
//     }

// private:
//     void thread_main() {
//         using namespace std::chrono_literals;

//         auto trajectory = Trajectory_V1();
//         auto tick       = std::chrono::steady_clock::now();

//         while (rclcpp::ok()) {
//             if (auto target = target_.load(std::memory_order_acquire)) {
//                 auto diff =
//                     std::chrono::steady_clock::now() -
//                     timestamp_.load(std::memory_order_acquire);
//                 if (diff > 500ms)
//                     continue;

//                 double fly_time = 0;
//                 for (int i = 5; i-- > 0;) {
//                     auto pos = target->Predict(
//                         static_cast<std::chrono::duration<double>>(diff).count() + fly_time +
//                         0.05);
//                     auto aiming_direction = *trajectory.GetShotVector(pos, 28.0, fly_time);
//                     if (i == 0) {
//                         auto gimbal_pose = transformer::GetTransform<GimbalGyro, GimbalLink>();
//                         auto delta_yaw =
//                             Eigen::AngleAxisd{0.000, gimbal_pose * Eigen::Vector3d::UnitZ()};
//                         auto delta_pitch =
//                             Eigen::AngleAxisd{0.005, gimbal_pose * Eigen::Vector3d::UnitY()};
//                         aiming_direction = (delta_pitch * (delta_yaw *
//                         (aiming_direction))).eval(); auto msg         =
//                         std::make_unique<geometry_msgs::msg::Vector3>(); msg->x           =
//                         aiming_direction.x(); msg->y           = aiming_direction.y(); msg->z =
//                         aiming_direction.z();
//                         aiming_direction_publisher_->publish(std::move(msg));
//                     }
//                 }
//             }

//             tick += 1ms;
//             std::this_thread::sleep_until(tick);
//         }
//     }

//     std::thread thread_;

//     std::atomic<TargetInterface*> target_ = nullptr;
//     std::atomic<std::chrono::steady_clock::time_point> timestamp_;

//     rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr gimbal_pose_subscription_;
//     rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr aiming_direction_publisher_;
// };

void GimbalInfantry::Always(
    TargetInterface*& target_ref, std::chrono::steady_clock::time_point& timestamp_ref,
    rmcs_executor::Component::InputInterface<rmcs_core::msgs::RoboticColor>& color,
    rmcs_executor::Component::InputInterface<uint8_t>& robot_id,
    std::chrono::milliseconds exposure_time,
    rmcs_executor::Component::InputInterface<bool>& buff_mode, int64_t armor_predict_duration,
    int64_t buff_predict_duration) {

    hikcamera::ImageCapturer::CameraProfile camera_profile;
    camera_profile.exposure_time = exposure_time;
    camera_profile.gain          = 16.9807;

    if ((*robot_id) == 7) {
        camera_profile.invert_image = true;
    } else {
        camera_profile.invert_image = false;
    }
    hikcamera::ImageCapturer image_capturer(camera_profile);

    // auto img_capture = HikCameraCapture();
    // auto img_capture = CVVideoCapture("./videos/test_vid.mp4");

    // auto cboard = CBoardInfantryAsync("/dev/IMU", "/dev/CBoard");

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("ugas");

    std::string path0     = package_share_directory + "/models/NumberIdentifyModelV4.pb";
    auto armor_identifier = ArmorIdentifier_V3<NumberIdentifier_V1>(path0.c_str());
    auto buff_identifier =
        BuffIdentifier_V1(package_share_directory + "/models/buff_nocolor_v6.onnx");

    // auto simple_predictor = SimplePredictor();
    auto ekf_tracker  = ArmorEKFTracker();
    auto buff_tracker = BuffTracker();

    // auto sender = TestSender();

    auto fps = FPSCounter_V2();

    auto recorder = PNGRecorder("images/", ENABLE_RECORDING ? 3.0 : 0.0);

    // bool autoscope_enabled = true;
    bool buff_enabled = false;

    while (rclcpp::ok()) {
        auto img       = image_capturer.read();
        auto timestamp = std::chrono::steady_clock::now();

        if constexpr (debugCanvas.master) {
            debugCanvas.master.LoadMat(img);
        }
        // autoscope_enabled = cboard.get_auto_scope_enabled();

        do {
            // if (!buff_enabled && cboard.get_buff_mode_enabled())
            // buff_tracker.ResetAll();
            // buff_enabled = cboard.get_buff_mode_enabled();
            if (!buff_enabled && *buff_mode) {
                buff_tracker.ResetAll();
                buff_enabled = *buff_mode;
            }

            if (!buff_enabled) {
                auto armors = armor_identifier.Identify(
                    img, *color == rmcs_core::msgs::RoboticColor::Blue ? ArmorColor::Red
                                                                       : ArmorColor::Blue);
                auto armors3d = ArmorPnPSolver::SolveAll(armors);
                if (auto target = ekf_tracker.Update(armors3d, timestamp, armor_predict_duration)) {
                    timestamp_ref = timestamp;
                    target_ref    = target.release();
                    // sender.update(std::move(target), timestamp);
                    // cboard.Send(std::move(target), timestamp);
                    break;
                }
            } else {
                if (auto buff = buff_identifier.Identify(img)) {
                    if (auto buff3d = BuffPnPSolver::Solve(*buff)) {
                        if (auto target =
                                buff_tracker.Update(*buff3d, timestamp, buff_predict_duration)) {
                            timestamp_ref = timestamp;
                            target_ref    = target.release();
                            // sender.update(std::move(target), timestamp);
                            // cboard.Send(std::move(target), timestamp);
                            break;
                        }
                    }
                }
            }
            // cboard.Send();
        } while (false);

        if constexpr (ENABLE_DEBUG_CANVAS) {
            static int interval = 0;
            if (interval-- == 0) {
                debugCanvas.ShowAll();
                cv::waitKey(1);
                interval = 20;
            }
        }

        if (fps.Count()) {
            RCLCPP_INFO(rclcpp::get_logger("ugas"), "Fps: %d", fps.GetFPS());
        }
    }
}

#pragma once
/*
Creation Date: 2023/05/27
Latest Update: 2023/05/27
Developer(s): 22-Qzh
Reference(s): ChenJun armor_tracker
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 基于EKF的目标追踪器
*/

#include <cmath>

#include <vector>
#include <optional>

#include <opencv2/opencv.hpp>

#include "Core/PnPSolver/ArmorPnPSolverInterface.h"
#include "Util/TimeStamp/TimeStampCounter.h"
#include "Util/Parameter/Parameters.h"


class ArmorEKFTracker {

public:
    struct Target {
    public:
        explicit Target(const Eigen::Vector3d& pos, double yaw, double v_yaw) : _pos(pos), _yaw(yaw), _v_yaw(v_yaw) { }

        Eigen::Vector3d Predict(float sec) const {
            return _pos;
        }

        bool Shotable(float sec) const {
            //double yaw_diff = std::fmod(_yaw + _v_yaw * sec, 2 * MathConsts::Pi / 6);

            constexpr double div = 2 * MathConsts::Pi / 3;
            double yaw_diff = std::fmod(_yaw + _v_yaw * sec, div);
            //std::cout << yaw_diff << '\n';
            if (yaw_diff < 0) yaw_diff += div;

            return (yaw_diff < 0.4 || yaw_diff > div - 0.4);
        }

    private:
        Eigen::Vector3d _pos;
        double _yaw, _v_yaw;
    };

private:
    class EKF {
    public:
        EKF() {
            Eigen::DiagonalMatrix<double, 9> p;
            p.setIdentity();
            _P = p;
        };
        EKF(const EKF& EKF) = delete;
        EKF(EKF&&) = delete;

        void predict(double t) {
            Eigen::MatrixXd F = jacobian_f(_x, t), Q = get_Q(t);

            _x = f(_x, t);
            _P = F * _P * F.transpose() + Q;
        }

        void update(const Eigen::VectorXd& z, double t) {
            Eigen::MatrixXd H = jacobian_h(_x), R = get_R(z);
            Eigen::MatrixXd I = Eigen::MatrixXd::Identity(9, 9);

            Eigen::MatrixXd K = _P * H.transpose() * (H * _P * H.transpose() + R).inverse();
            _x = _x + K * (z - h(_x));
            _P = (I - K * H) * _P;

            /*std::cout << "ababab\n";
            std::cout << K;
            std::this_thread::sleep_for(std::chrono::seconds(100));*/
        }

        Eigen::Matrix<double, 9, 1> _x;
        Eigen::Matrix<double, 9, 9> _P;

    private:
        static constexpr double _sigma2_q_xyz = 20.0;
        static constexpr double _sigma2_q_yaw = 100.0;
        static constexpr double _sigma2_q_r = 800.0;
        static constexpr double _r_xyz_factor = 0.05;
        static constexpr double _r_yaw = 0.02;


        // f - Process function
        static Eigen::VectorXd f(const Eigen::VectorXd& x, double dt) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt;
            x_new(2) += x(3) * dt;
            x_new(4) += x(5) * dt;
            x_new(6) += x(7) * dt;
            return x_new;
        }

        // J_f - Jacobian of process function
        static Eigen::MatrixXd jacobian_f(const Eigen::VectorXd& x, double dt) {
            Eigen::MatrixXd f(9, 9);
            f << 1, dt, 0, 0, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 1, dt, 0, 0, 0, 0, 0,
                0, 0, 0, 1, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 1, dt, 0, 0, 0,
                0, 0, 0, 0, 0, 1, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 1, dt, 0,
                0, 0, 0, 0, 0, 0, 0, 1, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 1;
            return f;
        };

        // h - Observation function
        static Eigen::VectorXd h(const Eigen::VectorXd& x) {
            Eigen::VectorXd z(4);
            double xc = x(0), yc = x(2), yaw = x(6), r = x(8);
            z(0) = xc - r * cos(yaw);  // xa
            z(1) = yc - r * sin(yaw);  // ya
            z(2) = x(4);                  // za
            z(3) = x(6);                  // yaw
            return z;
        };

        // jacobian_h - Jacobian of process function
        static Eigen::MatrixXd jacobian_h(const Eigen::VectorXd& x) {
            Eigen::MatrixXd h(4, 9);
            double yaw = x(6), r = x(8);
            // clang-format off
            //    xc   v_xc yc   v_yc za   v_za yaw         v_yaw r
            h << 1, 0, 0, 0, 0, 0, r* sin(yaw), 0, -cos(yaw),
                0, 0, 1, 0, 0, 0, -r * cos(yaw), 0, -sin(yaw),
                0, 0, 0, 0, 1, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 1, 0, 0;
            // clang-format on
            return h;
        };

        // Q - process noise covariance matrix
        static Eigen::MatrixXd get_Q(double dt) {
            Eigen::MatrixXd q(9, 9);
            double t = dt, x = _sigma2_q_xyz, y = _sigma2_q_yaw, r = _sigma2_q_r;
            double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
            double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * x, q_vy_vy = pow(t, 2) * y;
            double q_r = pow(t, 4) / 4 * r;
            // clang-format off
            //    xc      v_xc    yc      v_yc    za      v_za    yaw     v_yaw   r
            q << q_x_x, q_x_vx, 0, 0, 0, 0, 0, 0, 0,
                q_x_vx, q_vx_vx, 0, 0, 0, 0, 0, 0, 0,
                0, 0, q_x_x, q_x_vx, 0, 0, 0, 0, 0,
                0, 0, q_x_vx, q_vx_vx, 0, 0, 0, 0, 0,
                0, 0, 0, 0, q_x_x, q_x_vx, 0, 0, 0,
                0, 0, 0, 0, q_x_vx, q_vx_vx, 0, 0, 0,
                0, 0, 0, 0, 0, 0, q_y_y, q_y_vy, 0,
                0, 0, 0, 0, 0, 0, q_y_vy, q_vy_vy, 0,
                0, 0, 0, 0, 0, 0, 0, 0, q_r;
            // clang-format on
            return q;
        };

        // R - measurement noise covariance matrix
        static Eigen::DiagonalMatrix<double, 4> get_R(const Eigen::VectorXd& z) {
            Eigen::DiagonalMatrix<double, 4> r;
            double x = _r_xyz_factor;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), _r_yaw;
            return r;
        };

        // P - error estimate covariance matrix
        //static Eigen::DiagonalMatrix<double, 9> get_P() {
        //}
    };

    class Tracker {
    public:

        Tracker() = default;
        Tracker(const Tracker&) = delete;
        Tracker(Tracker&&) = delete;

        void Update(const ArmorPlate3d& armor3d, std::chrono::steady_clock::time_point timeStamp) {
            double yaw = getContinuousYaw(armor3d);
            double xa = armor3d.position.x() / 1000, ya = armor3d.position.y() / 1000, za = armor3d.position.z() / 1000;

            //std::cout << armor3d.normal.z() << '\n';

            double dt = std::chrono::duration<double>(timeStamp - _last_update).count();

            if (!_init || dt > 1.5) {
                //std::cout << "tracker reset\n";

                double r = -0.26;
                _another_r = r;
                double xc = xa + r * cos(yaw);
                double yc = ya + r * sin(yaw);

                // xc  v_xc  yc  v_yc  za  v_za  yaw  v_yaw  r
                _ekf._x << xc, 0, yc, 0, za, 0, yaw, 0, r;
                _init = true;
            }
            else {
                _ekf.predict(dt);

                //std::cout << xa << ' ' << ya << ' ' << za << ' ' << yaw << '\n';

                if ((_last_pos - Eigen::Vector3d{ xa, ya, za }).norm() > 0.20) {
                    std::cout << "Armor jump\n";
                    _ekf._x(4) = za;
                    _ekf._x(6) = yaw;
                    //std::swap(_ekf._x(8), _another_r);
                }
                else {
                    //std::cout << (getTrackingArmorPosFromEKF() - Eigen::Vector3d{ xa, ya, za }).norm() << '\n';
                    //if ((getTrackingArmorPosFromEKF() - Eigen::Vector3d{ xa, ya, za }).norm() > 0.15)
                    //    std::cout << "aaa\n";

                    Eigen::Vector4d measurement = { xa, ya, za, yaw };
                    _ekf.update(measurement, dt);
                    _ekf._x(1) = _ekf._x(3) = _ekf._x(5) = 0;

                    /*double r = 0.26;
                    double xc = xa + r * cos(yaw);
                    double yc = ya + r * sin(yaw);
                    _ekf._x << xc, 0, yc, 0, za, 0, yaw, 0, r;*/

                    /*for (int i = 0; i < 9; ++i)
                        std::cout << _ekf._x(i) << ' ';
                    std::cout << '\n';*/
                    //std::cout << _ekf._P;
                    //std::cout << "\n\n\n\n\n\n\n\n";

                    /*if (_ekf._x(8) < 0.12)
                        _ekf._x(8) = 0.12;
                    else if (_ekf._x(8) > 0.4)
                        _ekf._x(8) = 0.4;*/

                    if (_ekf._x(8) > -0.12)
                        _ekf._x(8) = -0.12;
                    else if (_ekf._x(8) < -0.4)
                        _ekf._x(8) = -0.4;
                }
            }

            _last_update = timeStamp;
            _last_pos = Eigen::Vector3d{ xa, ya, za };
        }

        void Update(std::chrono::steady_clock::time_point timeStamp) {
            double dt = std::chrono::duration<double>(timeStamp - _last_update).count();
            _ekf.predict(dt);
            _last_update = timeStamp;
        }

        Target GetTarget() {
            return getOutPostTarget();
        }

    private:
        double getContinuousYaw(const ArmorPlate3d& armor3d) {
            double yaw = atan2(armor3d.normal.y(), armor3d.normal.x());

            // Generate continuous yaw (-pi~pi -> -inf~inf)
            double diff = std::fmod(yaw - _last_yaw, 2 * MathConsts::Pi);
            if (diff < -MathConsts::Pi) diff += 2 * MathConsts::Pi;
            else if (diff > MathConsts::Pi) diff -= 2 * MathConsts::Pi;
            _last_yaw += diff;

            return _last_yaw;
        }

        Eigen::Vector3d getTrackingArmorPosFromEKF()
        {
            const Eigen::VectorXd& x = _ekf._x;
            double xc = x(0), yc = x(2), za = x(4);
            double yaw = x(6), r = x(8);
            double xa = xc - r * cos(yaw);
            double ya = yc - r * sin(yaw);
            return Eigen::Vector3d(xa, ya, za);
        }

        Target getOutPostTarget() {
            const Eigen::VectorXd& x = _ekf._x;
            double xc = x(0), yc = x(2), za = x(4);
            double aim_yaw = atan2(-yc, -xc);
            double yaw = x(6), v_yaw = x(7), r = x(8);
            double xa = xc - r * cos(aim_yaw);
            double ya = yc - r * sin(aim_yaw);

            double delta_yaw = std::fmod(yaw - aim_yaw, 2 * MathConsts::Pi / 3);
            if (delta_yaw < 0) delta_yaw += 2 * MathConsts::Pi / 3;

            // std::cout << yaw << '\n';
            // std::cout << delta_yaw << ' ' << v_yaw << '\n';

            return Target(Eigen::Vector3d(xa, ya, za) * 1000, delta_yaw, v_yaw);
        }

        bool _init = false;

        double _last_yaw = 0;
        double _another_r;

        EKF _ekf;
        std::chrono::steady_clock::time_point _last_update;
        Eigen::Vector3d _last_pos;
    };

public:
    ArmorEKFTracker() = default;
    ArmorEKFTracker(const ArmorEKFTracker&) = delete;
    ArmorEKFTracker(ArmorEKFTracker&&) = delete;

    template <typename TransformerType>
    std::optional<Target> Update(const std::vector<ArmorPlate3d>& armors3d, std::chrono::steady_clock::time_point timeStamp, const TransformerType& transformer) {

        std::vector<double> distances;
        std::vector<double> angles;
        for (const auto& armor3d : armors3d) {
            auto pos = transformer.Gyro2Link(armor3d.position);
            const double& x = pos.x(), & y = pos.y(), & z = pos.z();
            double dis = sqrt(x * x + y * y + z * z);
            distances.push_back(dis);
            double angle = x / dis;
            angles.push_back(angle);
        }

        for (const auto& armor3d : armors3d) {
            if (armor3d.id == ArmorID::Outpost) {
                lastTarget = ArmorID::Outpost;
                lastSuccess = timeStamp;
                break;
            }
            else if (armor3d.id == ArmorID::Hero) {
                lastTarget = ArmorID::Hero;
                lastSuccess = timeStamp;
                break;
            }
        }

        std::vector<int> sortedAngles;
        for (int i = 0; i < armors3d.size(); ++i)
            sortedAngles.push_back(i);
        std::sort(sortedAngles.begin(), sortedAngles.end(), [&angles](const int& a, const int& b) -> bool {
            return angles[a] < angles[b];
        });

        if (lastTarget != ArmorID::Unknown) {
            if (timeStamp - lastSuccess < std::chrono::milliseconds(1000)) {
                if (lastTarget == ArmorID::Outpost) {
                    for (const auto& i : sortedAngles)
                        if (armors3d[i].id == lastTarget) {
                            tracker.Update(armors3d[i], timeStamp);
                            return tracker.GetTarget();
                        }
                    tracker.Update(timeStamp);
                    return tracker.GetTarget();
                }
                else {
                    for (const auto& i : sortedAngles)
                        if (armors3d[i].id == lastTarget)
                            return Target(armors3d[i].position, 0, 0);
                }
            }
            else lastTarget = ArmorID::Unknown;
        }

        if (!armors3d.empty())
            return Target(armors3d[0].position, 0, 0);

        /*if (!armors3d.empty()) {
            auto& armor3d = armors3d[0];
            tracker.Update(armor3d, timeStamp);
            return tracker.GetTarget();
        }*/

        return {};

    }

public:
    ArmorID lastTarget = ArmorID::Unknown;
    std::chrono::steady_clock::time_point lastSuccess;

    Tracker tracker;
};

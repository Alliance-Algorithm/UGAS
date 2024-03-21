#include "ballistic_solver.hpp"

#include <eigen3/Eigen/Dense>

class BallisticSolver::Impl {
public:
    [[nodiscard]] static Eigen::Vector3d
        solve(Eigen::Vector3d target, Eigen::Vector3d muzzle, double speed) {
        auto pos = (target - muzzle).eval();

        const double& x = pos.x();
        const double& y = pos.y();
        const double& z = pos.z();

        double yaw   = atan2(y, x);
        double pitch = 0;

        double a = speed * speed; // v0 ^ 2
        double b = a * a;         // v0 ^ 4
        double c = x * x + y * y; // xt ^ 2
        double d = c * c;         // xt ^ 4
        double e = g * g;         // g ^ 2

        double xt = sqrt(c);      // target horizontal distance

        double f = b * d * (b - e * c - 2 * g * a * z);
        if (f >= 0) {
            pitch = -atan((b * c - sqrt(f)) / (g * a * c * xt));
        }

        // double fly_time = xt / (cos(pitch) * speed);

        return {cos(pitch) * cos(yaw), cos(pitch) * sin(yaw), -sin(pitch)};
    }

private:
    static constexpr double g = 9.81;
};

BallisticSolver::BallisticSolver() {}

Eigen::Vector3d
    BallisticSolver::solve(Eigen::Vector3d target, Eigen::Vector3d muzzle, double speed) const {
    return Impl::solve(target, muzzle, speed);
}

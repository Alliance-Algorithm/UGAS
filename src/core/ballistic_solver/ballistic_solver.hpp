#pragma once

#include <eigen3/Eigen/Dense>

class BallisticSolver {
public:
    BallisticSolver();

    [[nodiscard]] Eigen::Vector3d
        solve(Eigen::Vector3d target, Eigen::Vector3d muzzle, double speed) const;
        
private:
    class Impl;
};
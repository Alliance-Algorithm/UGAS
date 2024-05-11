#ifndef BUNDLE_ADJUSTMENT_H
#define BUNDLE_ADJUSTMENT_H

/**
 * @brief definition of error (constraint)
 * 
 * Using BA to estimate 2D pose of car (3) + 2D vel of car (3) + z + vel_z + radius
 * 
 * Using pixels as observations, and consider multiple frames
 * Ceres Solver
 */
struct ReprojectionError {
    ReprojectionError(int observed_x, int observed_y) : observed_x_(observed_x), observed_y_(observed_y) {}

    // TODO: define operator ()
    template <typename T>
    bool operator()() const {
        
    }

    int observed_x_;
    int observed_y_;

};

/**
 * @brief Solve
 * 
 */

#endif
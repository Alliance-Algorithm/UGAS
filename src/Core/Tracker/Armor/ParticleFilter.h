#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <eigen3/Eigen/Dense>
#include <vector>
#include <random>


/**
 * Particle Filter interface
*/
template<typename TYPE_STATE, typename TYPE_OBSERVATION>
class ParticleFilter {
protected:

public:
    // type definition

    // params
    struct Params {
        Params() {}

        // hyper parameters for PF
        size_t num_particles = 100;
        // noise parameters

    };

    ParticleFilter(Params params=Params()): params_(params) {

    }

    virtual ~ParticleFilter() {}

    virtual bool init() = 0; 
    //
    virtual bool predict(double delta_t) = 0;
    //
    virtual bool update(TYPE_OBSERVATION observation) = 0;

    virtual bool updateWeights() = 0;
    virtual bool resample() = 0;
    virtual void getEstimation(TYPE_STATE& state) = 0;

private:

    Params params_;

    // states, to be estimated

    // observations

};

/**
 * State definition
 * states to be estimated
 */
struct State {
    double pos_c_x; // x positon of car
    double pos_c_y;
    double pos_a_z; // z position of armor
    double vel_c_x;
    double vel_c_y;
    double vel_c_z;
    double yaw;
    double vel_yaw;
    double radius_c;
};

/**
 * Observation definition
 * state of armor is observed
 */
struct Observation {
    double pos_a_x;
    double pos_a_y;
    double pos_a_z;
    double yaw;
};

class ArmorParticleFilter : ParticleFilter<State, Observation> {
    
public:

private:

    State state_;
    Observation observation_;

};


#endif
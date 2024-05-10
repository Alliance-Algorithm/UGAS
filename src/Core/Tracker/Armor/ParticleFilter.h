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
    // Params params_;
    // random number generator
    std::default_random_engine gen_;
public:
    // type definition

    // params
    // struct Params {
    //     Params() {}

    //     // hyper parameters for PF
    //     size_t num_particles = 100;
    //     // noise parameters
    // };

    ParticleFilter() { }

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

    

    // states, to be estimated

    // observations

    
};

template<typename TYPE_STATE>
struct Particle {
    Particle(TYPE_STATE state, size_t i=0, double weight=1.0) : particle_state{state}, id{i}, particle_weight{weight} {}
    // Particle(TYPE_STATE&& state, size_t i=0, double weight=1.0) : particle_state{state}, id{i}, particle_weight{weight} {}

    TYPE_STATE particle_state;
    double particle_weight;
    size_t id;
};

/**
 * State definition
 * states to be estimated
 */
struct StateType {
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
struct ObservationType {
    double pos_a_x;
    double pos_a_y;
    double pos_a_z;
    double yaw;
};

using StateNoiseType = StateType;
using ObservationNoiseType = ObservationType;

/**
 * 
 * TODO
 */
class ArmorParticleFilter : ParticleFilter<StateType, ObservationType> {
    
public:
    // params
    struct Params {
        Params() {}

        // hyper parameters for PF
        size_t num_particles = 30;
        // noise parameters
        double pos_c_x_std = 0.1;
        double pos_c_y_std = 0.1; // standard deviation of y position of center
        double pos_a_z_std = 0.1; // standard deviation of z position of armor
        double vel_c_x_std = 0.1; // standard deviation of velocity along x-axis of center
        double vel_c_y_std = 0.1; // standard deviation of velocity along y-axis of center
        double vel_c_z_std = 0.1; // standard deviation of velocity along z-axis of center
        double yaw_std = 0.1; // standard deviation of yaw angle
        double vel_yaw_std = 0.1; // standard deviation of angular velocity (yaw rate)
        double radius_c_std = 0.1; // standard deviation of radius of center

        double observed_pos_a_x_std = 0.1; // standard deviation of x position of armor
        double observed_pos_a_y_std = 0.1; // standard deviation of y position of armor
        double observed_pos_a_z_std = 0.1; // standard deviation of z position of armor
        double observed_yaw_armor_std = 0.1; // standard deviation of yaw angle of armor
    };

    ArmorParticleFilter(Params params=Params()) : params_{params}, 
                                                is_initialized_{false},
                                                init_state_noise_std_{      // TODO: this is not used
                                                    params_.pos_c_x_std,
                                                    params_.pos_c_y_std,
                                                    params_.pos_a_z_std,
                                                    params_.vel_c_x_std,
                                                    params_.vel_c_y_std,
                                                    params_.vel_c_z_std,
                                                    params_.yaw_std,
                                                    params_.vel_yaw_std,
                                                    params_.radius_c_std} ,
                                                motion_noise_std_{
                                                    params_.pos_c_x_std,
                                                    params_.pos_c_y_std,
                                                    params_.pos_a_z_std,
                                                    params_.vel_c_x_std,
                                                    params_.vel_c_y_std,
                                                    params_.vel_c_z_std,
                                                    params_.yaw_std,
                                                    params_.vel_yaw_std,
                                                    params_.radius_c_std} ,
                                                observation_noise_std_{
                                                    params_.observed_pos_a_x_std,
                                                    params_.observed_pos_a_y_std,
                                                    params_.observed_pos_a_z_std,
                                                    params_.observed_yaw_armor_std
                                                }
    {

        particles_.reserve(params_.num_particles);
    }

    bool init(const StateType& init_state, const StateNoiseType& init_state_std) {

        std::normal_distribution<double> dist_pos_c_x{init_state.pos_c_x, init_state_std.pos_c_x};
        std::normal_distribution<double> dist_pos_c_y{init_state.pos_c_y, init_state_std.pos_c_y};
        std::normal_distribution<double> dist_pos_a_z{init_state.pos_a_z, init_state_std.pos_a_z};
        std::normal_distribution<double> dist_vel_c_x{init_state.vel_c_x, init_state_std.vel_c_x};
        std::normal_distribution<double> dist_vel_c_y{init_state.vel_c_y, init_state_std.vel_c_y};
        std::normal_distribution<double> dist_vel_c_z{init_state.vel_c_z, init_state_std.vel_c_z};
        std::normal_distribution<double> dist_yaw{init_state.yaw, init_state_std.yaw};
        std::normal_distribution<double> dist_vel_yaw{init_state.vel_yaw, init_state_std.vel_yaw};
        std::normal_distribution<double> dist_radius_c{init_state.radius_c, init_state_std.radius_c};
        
        for (int id_particle = 0; id_particle < params_.num_particles; id_particle++) {
            StateType state{dist_pos_c_x(gen_),
                            dist_pos_c_y(gen_),
                            dist_pos_a_z(gen_),
                            dist_vel_c_x(gen_),
                            dist_vel_c_y(gen_),
                            dist_vel_c_z(gen_),
                            dist_yaw(gen_),
                            dist_vel_yaw(gen_),
                            dist_radius_c(gen_)};
            particles_[id_particle] = Particle<StateType>{std::move(state), id_particle, 1.0}; // TODO: avoid copying
        }
        is_initialized_ = true;
        return is_initialized_;
    }

    bool predict(double delta_t) {
        // update the state of each particle
        for (auto &particle: particles_) {
            // apply motion model
            applyMotionModel(particle, delta_t);
           
            // add random noise
            std::normal_distribution<double> dist_pos_c_x{particle.particle_state.pos_c_x, motion_noise_std_.pos_c_x};
            std::normal_distribution<double> dist_pos_c_y{particle.particle_state.pos_c_y, motion_noise_std_.pos_c_y};
            std::normal_distribution<double> dist_pos_a_z{particle.particle_state.pos_a_z, motion_noise_std_.pos_a_z};
            std::normal_distribution<double> dist_vel_c_x{particle.particle_state.vel_c_x, motion_noise_std_.vel_c_x};
            std::normal_distribution<double> dist_vel_c_y{particle.particle_state.vel_c_y, motion_noise_std_.vel_c_y};
            std::normal_distribution<double> dist_vel_c_z{particle.particle_state.vel_c_z, motion_noise_std_.vel_c_z};
            std::normal_distribution<double> dist_yaw{particle.particle_state.yaw, motion_noise_std_.yaw};
            std::normal_distribution<double> dist_vel_yaw{particle.particle_state.vel_yaw, motion_noise_std_.vel_yaw};
            std::normal_distribution<double> dist_radius_c{particle.particle_state.radius_c, motion_noise_std_.radius_c};

            particle.particle_state.pos_c_x = dist_pos_c_x(gen_);
            particle.particle_state.pos_c_y = dist_pos_c_y(gen_);
            particle.particle_state.pos_a_z = dist_pos_a_z(gen_);
            particle.particle_state.vel_c_x = dist_vel_c_x(gen_);
            particle.particle_state.vel_c_y = dist_vel_c_y(gen_);
            particle.particle_state.vel_c_z = dist_vel_c_z(gen_);
            particle.particle_state.yaw = dist_yaw(gen_);
            particle.particle_state.vel_yaw = dist_vel_yaw(gen_);
            particle.particle_state.radius_c = dist_radius_c(gen_);
        }
        return true;
    }
    // //
    bool update(ObservationType observation) {

    }

    bool updateWeights(const std::vector<ObservationType>& observations) {
        
        for (auto &particle: particles_) {
            /**
             * Apply observation model to each particle
             */
            ObservationType deduced_observation;
            applyObservationModel(particle, deduced_observation);

            /**
             * Calculate error of each particle
             */
            double error = 0.0; // TODO

            /**
             * Calculate weight of each particle using Gaussian distribution
             */


            /**
             * Normalize the weights to [0~1]
             */
            
        }
    }
    // bool resample() = 0;
    // void getEstimation(TYPE_STATE& state) = 0;

    /**
     * 
     */
    void applyMotionModel(Particle<StateType>& particle, double delta_t) {
        particle.particle_state.pos_c_x += particle.particle_state.vel_c_x * delta_t;
        particle.particle_state.pos_c_y += particle.particle_state.vel_c_y * delta_t;
        particle.particle_state.pos_a_z += particle.particle_state.vel_c_z * delta_t;
        particle.particle_state.yaw     += particle.particle_state.vel_yaw * delta_t;
    }

    /**
     * 
     */
    void applyObservationModel(const Particle<StateType>& particle, ObservationType& deduced_observation) {
        deduced_observation.pos_a_x = particle.particle_state.pos_c_x - 
                                        particle.particle_state.radius_c * cos(particle.particle_state.yaw);
        deduced_observation.pos_a_y = particle.particle_state.pos_c_y - 
                                        particle.particle_state.radius_c * sin(particle.particle_state.yaw);
        deduced_observation.pos_a_z = particle.particle_state.pos_a_z;
        deduced_observation.yaw     = particle.particle_state.yaw;
    }

    /**
     * Distance, i.e., observation_l - observation_r
     * TODO: how to balance xyz and yaw ?
     */
    double distance(const ObservationType& observation_l, const ObservationType& observation_r) {


        return 0.0;
    }

private:

    Params params_;
    // StateType state_;
    StateNoiseType init_state_noise_std_;   // noise std of initial state
    StateNoiseType motion_noise_std_;       // noise std of motion
    ObservationNoiseType observation_noise_std_;    // noise std of observation

    // ObservationType observation_;

    std::vector<Particle<StateType>> particles_; 


    bool is_initialized_;

};


#endif
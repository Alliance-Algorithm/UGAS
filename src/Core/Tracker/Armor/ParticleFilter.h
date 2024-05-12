#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <eigen3/Eigen/Dense>
#include <vector>
#include <random>
#include <cmath>
#include <stdexcept>

using FLOAT_TYPE = double;

/**
 * Particle Filter interface
*/
template<typename TYPE_STATE, typename TYPE_OBSERVATION>
class ParticleFilter {
protected:
    // random number generator
    std::default_random_engine gen_;
public:

    ParticleFilter() { }

    virtual ~ParticleFilter() {}

    virtual bool init() = 0; 
    //
    virtual bool predict(FLOAT_TYPE delta_t) = 0;
    //
    virtual bool update(TYPE_OBSERVATION observation) = 0;

    virtual bool updateWeights() = 0;
    virtual bool resample() = 0;
    virtual void calculateEstimation() = 0;
    virtual TYPE_STATE getEstimation() const = 0;

};

template<typename TYPE_STATE>
struct Particle {
    Particle(TYPE_STATE state, size_t i=0, FLOAT_TYPE weight=1.0) : particle_state{state}, id{i}, particle_weight{weight} {}
    // Particle(TYPE_STATE&& state, size_t i=0, FLOAT_TYPE weight=1.0) : particle_state{state}, id{i}, particle_weight{weight} {}

    TYPE_STATE particle_state;
    FLOAT_TYPE particle_weight;
    size_t id;
};

/**
 * State definition
 * states to be estimated
 */
struct StateType {
    StateType() {}
    StateType(const Eigen::Matrix<FLOAT_TYPE, 9, 1>& state_eigen) :
        pos_c_x{state_eigen(0)}, 
        pos_c_y{state_eigen(1)},
        pos_a_z{state_eigen(2)}, 
        vel_c_x{state_eigen(3)},
        vel_c_y{state_eigen(4)},
        vel_c_z{state_eigen(5)},
        yaw{state_eigen(6)},
        vel_yaw{state_eigen(7)},
        radius_c{state_eigen(8)}
    { }
    StateType& operator=(const Eigen::Matrix<FLOAT_TYPE, 9, 1>& state_eigen) {
        pos_c_x = state_eigen(0);
        pos_c_y = state_eigen(1);
        pos_a_z = state_eigen(2);
        vel_c_x = state_eigen(3);
        vel_c_y = state_eigen(4);
        vel_c_z = state_eigen(5);
        yaw = state_eigen(6);
        vel_yaw = state_eigen(7);
        radius_c = state_eigen(8);
        return *this;
    }    

    Eigen::Matrix<FLOAT_TYPE, 9, 1>& toEigen() {
        return {pos_c_x, pos_c_y, pos_a_z, vel_c_x, vel_c_y, vel_c_z, yaw, vel_yaw, radius_c};
    }

    FLOAT_TYPE pos_c_x; // x positon of car
    FLOAT_TYPE pos_c_y;
    FLOAT_TYPE pos_a_z; // z position of armor
    FLOAT_TYPE vel_c_x;
    FLOAT_TYPE vel_c_y;
    FLOAT_TYPE vel_c_z;
    FLOAT_TYPE yaw;
    FLOAT_TYPE vel_yaw;
    FLOAT_TYPE radius_c;
};

/**
 * Observation definition
 * state of armor is observed
 */
struct ObservationType {
    ObservationType() {}
    ObservationType(const Eigen::matrix<FLOAT_TYPE, 4, 1>& observation_eigen) :
        pos_a_x{observation_eigen(0)},
        pos_a_y{observation_eigen(1)},
        pos_a_z{observation_eigen(2)},
        yaw{observation(3)}
    { }

    Eigen::Matrix<FLOAT_TYPE, 4, 1>& toEigen() {
        return {pos_a_x, pos_a_y, pos_a_z, yaw};
    }

    FLOAT_TYPE pos_a_x;
    FLOAT_TYPE pos_a_y;
    FLOAT_TYPE pos_a_z;
    FLOAT_TYPE yaw;
};

enum class ResamplingMethod {
    Systematic,
    // TODO:
    // Multinomial,
    // Stratified,
    // Residual,
    // StochasticUniversalSampling
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
        FLOAT_TYPE pos_c_x_std = 0.1;
        FLOAT_TYPE pos_c_y_std = 0.1; // standard deviation of y position of center
        FLOAT_TYPE pos_a_z_std = 0.1; // standard deviation of z position of armor
        FLOAT_TYPE vel_c_x_std = 0.1; // standard deviation of velocity along x-axis of center
        FLOAT_TYPE vel_c_y_std = 0.1; // standard deviation of velocity along y-axis of center
        FLOAT_TYPE vel_c_z_std = 0.1; // standard deviation of velocity along z-axis of center
        FLOAT_TYPE yaw_std = 0.1; // standard deviation of yaw angle
        FLOAT_TYPE vel_yaw_std = 0.1; // standard deviation of angular velocity (yaw rate)
        FLOAT_TYPE radius_c_std = 0.1; // standard deviation of radius of center

        FLOAT_TYPE observed_pos_a_x_std = 0.1; // standard deviation of x position of armor
        FLOAT_TYPE observed_pos_a_y_std = 0.1; // standard deviation of y position of armor
        FLOAT_TYPE observed_pos_a_z_std = 0.1; // standard deviation of z position of armor
        FLOAT_TYPE observed_yaw_armor_std = 0.1; // standard deviation of yaw angle of armor
        ResamplingMethod resamplingMethod = ResamplingMethod::Systematic;
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

    /**
     * @brief 
     * 
     * @param init_state 
     * @param init_state_std 
     * @return true 
     * @return false 
     */
    bool init(const StateType& init_state, const StateNoiseType& init_state_std) {

        std::normal_distribution<FLOAT_TYPE> dist_pos_c_x{init_state.pos_c_x, init_state_std.pos_c_x};
        std::normal_distribution<FLOAT_TYPE> dist_pos_c_y{init_state.pos_c_y, init_state_std.pos_c_y};
        std::normal_distribution<FLOAT_TYPE> dist_pos_a_z{init_state.pos_a_z, init_state_std.pos_a_z};
        std::normal_distribution<FLOAT_TYPE> dist_vel_c_x{init_state.vel_c_x, init_state_std.vel_c_x};
        std::normal_distribution<FLOAT_TYPE> dist_vel_c_y{init_state.vel_c_y, init_state_std.vel_c_y};
        std::normal_distribution<FLOAT_TYPE> dist_vel_c_z{init_state.vel_c_z, init_state_std.vel_c_z};
        std::normal_distribution<FLOAT_TYPE> dist_yaw{init_state.yaw, init_state_std.yaw};
        std::normal_distribution<FLOAT_TYPE> dist_vel_yaw{init_state.vel_yaw, init_state_std.vel_yaw};
        std::normal_distribution<FLOAT_TYPE> dist_radius_c{init_state.radius_c, init_state_std.radius_c};
        
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

    /**
     * @brief 
     * 
     * @param delta_t 
     * @return true 
     * @return false 
     */
    bool predict(FLOAT_TYPE delta_t) {
        // update the state of each particle
        for (auto &particle: particles_) {
            // apply motion model
            applyMotionModel(particle, delta_t);
           
            // add random noise
            std::normal_distribution<FLOAT_TYPE> dist_pos_c_x{particle.particle_state.pos_c_x, motion_noise_std_.pos_c_x};
            std::normal_distribution<FLOAT_TYPE> dist_pos_c_y{particle.particle_state.pos_c_y, motion_noise_std_.pos_c_y};
            std::normal_distribution<FLOAT_TYPE> dist_pos_a_z{particle.particle_state.pos_a_z, motion_noise_std_.pos_a_z};
            std::normal_distribution<FLOAT_TYPE> dist_vel_c_x{particle.particle_state.vel_c_x, motion_noise_std_.vel_c_x};
            std::normal_distribution<FLOAT_TYPE> dist_vel_c_y{particle.particle_state.vel_c_y, motion_noise_std_.vel_c_y};
            std::normal_distribution<FLOAT_TYPE> dist_vel_c_z{particle.particle_state.vel_c_z, motion_noise_std_.vel_c_z};
            std::normal_distribution<FLOAT_TYPE> dist_yaw{particle.particle_state.yaw, motion_noise_std_.yaw};
            std::normal_distribution<FLOAT_TYPE> dist_vel_yaw{particle.particle_state.vel_yaw, motion_noise_std_.vel_yaw};
            std::normal_distribution<FLOAT_TYPE> dist_radius_c{particle.particle_state.radius_c, motion_noise_std_.radius_c};

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
    
    /**
     * @brief 
     * 
     * @param observation 
     * @return true 
     * @return false 
     */
    bool update(const ObservationType& observation) {
        updateWeights(observation);
        resample();
        calculateEstimation();
    }

    /**
     * @brief 
     * 
     * @param observations 
     * @return true 
     * @return false 
     */
    bool updateWeights(const ObservationType& observation) {
        
        FLOAT_TYPE weight_sum = 0.0;
        observation_ = observation;

        for (auto &particle: particles_) {
            /**
             * Apply observation model to each particle
             */
            ObservationType deduced_observation;
            applyObservationModel(particle, deduced_observation);

            /**
             * Calculate weight of each particle using Gaussian distribution
             */
            particle.weight *= gaussianProbability(deduced_observation, observation, params_.observation_noise_std); 
            // TODO: Gaussian distribution with dim=4
            weight_sum += particle.weight;
        }
        /**
         * Normalize the weights to [0~1]
         */
        for (int id_particle = 0; id_particle < particles_.size(); ++id_particle) {
            particles_[id_particle].weight /= weight_sum;
        }
        return true;
    }

    /**
     * @brief Resample
     * update particles_
     * 
     * @return true 
     * @return false 
     */
    bool resample() {
        switch(params_.resamplingMethod) {
            case ResamplingMethod::Systematic:
                systematicResampling(params_.num_particles);
            break;
            // TODO
            default:
                systematicResampling(params_.num_particles);
        }
        return true;
    }


    /**
     * @brief Systematic resampling method.
     *
     * @param particles_ori Particles before resampling.
     * @param weights_ori_norm Normalized weights before resampling.
     * @param particles_resampled Particles after resampling.
     * @param weights_resampled Weights after resampling.
     * @param N_r Number of particles to resample.
     */
    void systematicResampling(uint32_t N_r)
    {
        uint32_t N = params_.num_particles;
        std::vector<Particle<StateType>> new_particles = std::vector<Particle<StateType>>(N);

        Eigen::VectorXd weights_cum_sum = weightsCumSum();

        uint32_t id_particle = 0;

        // produces random values u0, uniformly distributed on the interval [0.0, 1.0 / N_r)
        // std::random_device rd;
        // std::mt19937 gen(rd());
        std::uniform_real_distribution<> uniform_dist(0.0, 1 / N_r);        // random real num between [0.0, 1/N_r)
        double u0 = uniform_dist(gen_);

        for (size_t id_new_particle = N - N_r; id_new_particle < N; ++id_new_particle)
        {
            // calculate u = u0 + (id_new_particle - (N - N_r)) / N_r
            double u = u0 + (id_new_particle - (N - N_r)) / N_r;

            // select the resampled particle
            while (weights_cum_sum(id_particle) < u)
                ++id_particle;

            // set new particles
            new_particles[id_new_particle] = particles_[id_particle];
            new_particles[id_new_particle].id = id_new_particle;
            new_particles[id_new_particle].particle_weight = 1 / N;
            // particles_resampled(id_new_particle) = particles_ori(id_particle);
            // weights_resampled(id_new_particle) = 1 / N;
        }
        particles_ = new_particles;
    }


    void calculateEstimation() {
        // TODO: maybe easier using Eigen matrix calculation
        // transform to eigen type
        // Eigen matrix: particles states
        // Eigen col vector: particles weights
        Eigen::Matrix<FLOAT_TYPE, 9, params_.num_particles> particles_state_matrix;
        Eigen::Matrix<FLOAT_TYPE, params_.num_particles, 1> particles_weights_vector;

        for (size_t id_particle = 0; id_particle < params_.num_particles; ++i) {
            particles_state_matrix.col(id_particle) = particles_[id_particle].particle_state.toEigen();
            particles_weights_vector(id_particle) = particles_[id_particle].particle_state;
        }

        state_ = particles_state_matrix * particles_weights_vector;
        // bad method
        // for (auto particle : particles_) {
        //     state.pos_c_x += particle.particle_weight * particle.particle_state.pos_c_x;
        //     state.pos_c_y += particle.particle_weight * particle.particle_state.pos_c_y;
        // }
    }

    StateType getEstimation() const {
        return state_;
    }

    /**
     * @brief 
     * 
     * @param particle 
     * @param delta_t 
     */
    void applyMotionModel(Particle<StateType>& particle, FLOAT_TYPE delta_t) {
        particle.particle_state.pos_c_x += particle.particle_state.vel_c_x * delta_t;
        particle.particle_state.pos_c_y += particle.particle_state.vel_c_y * delta_t;
        particle.particle_state.pos_a_z += particle.particle_state.vel_c_z * delta_t;
        particle.particle_state.yaw     += particle.particle_state.vel_yaw * delta_t;
    }

    /**
     * @brief 
     * 
     * @param particle 
     * @param deduced_observation 
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
     * @brief 
     * 
     * @param x 
     * @param mu 
     * @param sigma 
     * @return FLOAT_TYPE 
     */
    FLOAT_TYPE gaussianProbability(const Eigen::Matrix<FLOAT_TYPE, Dynamic, 1>& x, const Eigen::Matrix<FLOAT_TYPE, Dynamic, 1>& mu, const Eigen::MatrixXd& sigma) {
        if (x.size() != mu.size() || x.size() != sigma.rows() || sigma.rows() != sigma.cols()) {
            throw std::invalid_argument("Input dimensions do not match");
        }
        
        int n = x.size(); // Dimensionality
        FLOAT_TYPE det_sigma = sigma.determinant();
        if (det_sigma <= 0) {
            throw std::runtime_error("Sigma matrix is not positive definite");
        }

        FLOAT_TYPE exponent = 0.0;
        FLOAT_TYPE normalization_factor = pow(2 * M_PI, -n / 2.0);

        // Calculate exponent term in the Gaussian probability density function
        Eigen::VectorXd diff = x - mu;
        exponent = -0.5 * diff.transpose() * sigma.inverse() * diff;

        // Calculate the probability density
        return normalization_factor * exp(exponent) / sqrt(det_sigma);
    }

    /**
     * @brief Calculate cumulative sum of normalized weights of particles.
     * 
     * @return Eigen::VectorXd Cumulative sum of normalized weights of particles.
     */
    inline Eigen::VectorXd weightsCumSum()
    {
        uint32_t N = params_.num_particles;
        Eigen::VectorXd weights_cum_sum(N);

        weights_cum_sum(0) = particles_[0].particle_weight;
        for (size_t i = 1; i < N; ++i)
            weights_cum_sum(i) = weights_cum_sum(i - 1) + particles_[i].particle_weight;
        weights_cum_sum(N - 1) = 1.0;

        return weights_cum_sum;
    }


private:

    Params params_;
    StateType state_;
    StateNoiseType init_state_noise_std_;   // noise std of initial state
    StateNoiseType motion_noise_std_;       // noise std of motion
    ObservationNoiseType observation_noise_std_;    // noise std of observation

    ObservationType observation_;

    std::vector<Particle<StateType>> particles_; 


    bool is_initialized_;
};

/**
 * TODO
 * transform data to Eigen or from Eigen
 */


#endif
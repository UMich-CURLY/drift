#include "state_estimator.h"

using namespace inekf;

StateEstimator::StateEstimator(NoiseParams params, ErrorType error_type)
    : params_(params), error_type_(error_type) {}

void StateEstimator::set_state(RobotState& state) { state_ = state; }

const RobotState StateEstimator::get_state() const { return state_; }

template<typename imu_q_t>
void StateEstimator::add_imu_propagation(std::shared_ptr<imu_q_t> buffer_ptr,
                                         const bool estimate_bias) {
  propagation_ = std::make_shared<ImuPropagation<imu_q_t>>(
      buffer_ptr, params_, error_type_, estimate_bias);
}

template<typename kinematic_q_t>
void StateEstimator::add_kinematics_correction(
    std::shared_ptr<kinematic_q_t> buffer_ptr) {
  int aug_map_idx = state_.add_augmented_map();
  std::shared_ptr<Correction> correction
      = std::make_shared<KinematicsCorrection<kinematic_q_t>>(
          buffer_ptr, error_type_, aug_map_idx);
  corrections_.push_back(correction);
}

template<typename velocity_q_t>
void StateEstimator::add_velocity_correction(
    std::shared_ptr<velocity_q_t> buffer_ptr,
    const Eigen::Matrix3d& covariance) {
  std::shared_ptr<Correction> correction
      = std::make_shared<VelocityCorrection<velocity_q_t>>(
          buffer_ptr, error_type_, covariance);
  corrections_.push_back(correction);
}

void StateEstimator::run(double dt) {
  propagation_.get()->Propagate(state_, dt);
  for (auto correction : corrections_) {
    correction.get()->Correct(state_);
  }
}
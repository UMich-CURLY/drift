#include "state_estimator.h"

using namespace inekf;

StateEstimator::StateEstimator(NoiseParams params, ErrorType error_type)
    : params_(params), error_type_(error_type) {}

void StateEstimator::set_state(RobotState& state) { state_ = state; }

const RobotState StateEstimator::get_state() const { return state_; }

void StateEstimator::add_imu_propagation(
    std::shared_ptr<std::queue<ImuMeasurement<double>>> buffer_ptr,
    const bool estimate_bias) {
  propagation_ = std::make_shared<ImuPropagation>(buffer_ptr, params_,
                                                  error_type_, estimate_bias);
}

void StateEstimator::add_kinematics_correction(
    std::shared_ptr<std::queue<KinematicsMeasurement<double>>> buffer_ptr,
    const std::string& aug_type) {
  std::shared_ptr<Correction> correction
      = std::make_shared<KinematicsCorrection>(buffer_ptr, error_type_,
                                               aug_type);
  corrections_.push_back(correction);
}

void StateEstimator::add_velocity_correction(
    std::shared_ptr<std::queue<VelocityMeasurement<double>>> buffer_ptr,
    const Eigen::Matrix3d& covariance) {
  std::shared_ptr<Correction> correction = std::make_shared<VelocityCorrection>(
      buffer_ptr, error_type_, covariance);
  corrections_.push_back(correction);
}

void StateEstimator::run(double dt) {
  propagation_.get()->Propagate(state_, dt);
  for (auto correction : corrections_) {
    correction.get()->Correct(state_);
  }
}
#include "state_estimator.h"


StateEstimator::StateEstimator(NoiseParams params, ErrorType error_type)
    : params_(params), error_type_(error_type) {}

void StateEstimator::run_once() {
  propagation_.get()->Propagate(state_);
  std::cout << "After prop:\n" << state_.get_X() << std::endl;
  for (auto correction : corrections_) {
    correction.get()->Correct(state_);
  }
}

void StateEstimator::set_state(RobotState& state) { state_ = state; }

const RobotState StateEstimator::get_state() const { return state_; }

void StateEstimator::add_imu_propagation(IMUQueuePtr buffer_ptr,
                                         const bool estimate_bias) {
  propagation_ = std::make_shared<ImuPropagation>(buffer_ptr, params_,
                                                  error_type_, estimate_bias);
}

void StateEstimator::add_kinematics_correction(KinematicsQueuePtr buffer_ptr,
                                               const std::string& aug_type) {
  std::shared_ptr<Correction> correction
      = std::make_shared<KinematicsCorrection>(buffer_ptr, error_type_,
                                               aug_type);
  corrections_.push_back(correction);
}

void StateEstimator::add_velocity_correction(
    VelocityQueuePtr buffer_ptr, const Eigen::Matrix3d& covariance) {
  std::shared_ptr<Correction> correction = std::make_shared<VelocityCorrection>(
      buffer_ptr, error_type_, covariance);
  corrections_.push_back(correction);
}

// void BodyEstimator::initState(
//     const ImuMeasurement<double>& imu_packet_in,
//     const JointStateMeasurement& joint_state_packet_in, HuskyState& state) {
//   // Clear filter
//   // filter_.clear();

//   // Initialize state mean
//   Eigen::Quaternion<double> quat(
//       imu_packet_in.orientation.w, imu_packet_in.orientation.x,
//       imu_packet_in.orientation.y, imu_packet_in.orientation.z);
//   // Eigen::Matrix3d R0 = quat.toRotationMatrix(); // Initialize based on
//   // VectorNav estimate
//   Eigen::Matrix3d R0;

//   if (use_imu_ori_est_init_bias_) {
//     R0 = quat.toRotationMatrix();
//   } else {
//     R0 = Eigen::Matrix3d::Identity();
//   }

//   Eigen::Vector3d v0_body = joint_state_packet_in.getBodyLinearVelocity();
//   Eigen::Vector3d v0 = R0 * v0_body;    // initial velocity

//   // Eigen::Vector3d v0 = {0.0,0.0,0.0};
//   Eigen::Vector3d p0
//       = {0.0, 0.0, 0.0};    // initial position, we set imu frame as world
//   frame

//       inekf::RobotState initial_state;
//   initial_state.setRotation(R0);
//   initial_state.setVelocity(v0);
//   initial_state.setPosition(p0);
//   initial_state.setGyroscopeBias(bg0_);
//   initial_state.setAccelerometerBias(ba0_);

//   // Initialize state covariance
//   initial_state.setRotationCovariance(0.03 * Eigen::Matrix3d::Identity());
//   initial_state.setVelocityCovariance(0.01 * Eigen::Matrix3d::Identity());
//   initial_state.setPositionCovariance(0.00001 * Eigen::Matrix3d::Identity());
//   initial_state.setGyroscopeBiasCovariance(0.0001
//                                            * Eigen::Matrix3d::Identity());
//   initial_state.setAccelerometerBiasCovariance(0.0025
//                                                * Eigen::Matrix3d::Identity());

//   filter_.setState(initial_state);
//   std::cout << "Robot's state mean is initialized to: \n";
//   std::cout << filter_.getState() << std::endl;
//   std::cout << "Robot's state covariance is initialized to: \n";
//   std::cout << filter_.getState().getP() << std::endl;

//   // Set enabled flag
//   t_prev_ = imu_packet_in.getTime();
//   state.setTime(t_prev_);
//   imu_prev_ << imu_packet_in.angular_velocity.x,
//       imu_packet_in.angular_velocity.y, imu_packet_in.angular_velocity.z;
//   imu_packet_in.linear_acceleration.x, imu_packet_in.linear_acceleration.y,
//       imu_packet_in.linear_acceleration.z;

//   enabled_ = true;
// }

void StateEstimator::initStateByImuAndVelocity() {
  /// TODO: Implement clear filter
  // Clear filter
  // filter_.clear();

  // Initialize state mean
  std::shared_ptr<ImuPropagation> imu_propagation_ptr
      = std::dynamic_pointer_cast<ImuPropagation>(propagation_);
  const IMUQueuePtr imu_queue_ptr
      = imu_propagation_ptr.get()->get_sensor_data_buffer_ptr();
  const ImuMeasurement<double>& imu_packet_in = *(imu_queue_ptr->front().get());
  // imu_queue_ptr->pop();
  Eigen::Quaternion<double> quat = imu_packet_in.get_quaternion();
  // Eigen::Matrix3d R0 = quat.toRotationMatrix(); // Initialize based on
  // VectorNav estimate
  Eigen::Matrix3d R0 = Eigen::Matrix3d::Identity();

  Eigen::Vector3d v0_body;
  for (auto& correction : corrections_) {
    /// TODO: How to deal with multiple velocity measurements?
    if (correction.get()->get_correction_type() == CorrectionType::VELOCITY) {
      std::shared_ptr<VelocityCorrection> velocity_correction_ptr
          = std::dynamic_pointer_cast<VelocityCorrection>(correction);
      const VelocityQueuePtr velocity_queue_ptr
          = velocity_correction_ptr.get()->get_sensor_data_buffer_ptr();
      const VelocityMeasurement<double>& velocity_packet_in
          = *(velocity_queue_ptr->front().get());
      // velocity_queue_ptr->pop();
      v0_body = velocity_packet_in.get_velocity();
      break;
    }
  }

  Eigen::Vector3d v0 = R0 * v0_body;    // initial velocity

  Eigen::Vector3d p0
      = {0.0, 0.0, 0.0};    // initial position, we set imu frame as world frame

  R0 = Eigen::Matrix3d::Identity();
  RobotState initial_state;

  Eigen::Vector3d bg0 = imu_propagation_ptr.get()->get_estimate_gyro_bias();
  Eigen::Vector3d ba0 = imu_propagation_ptr.get()->get_estimate_accel_bias();

  initial_state.set_rotation(R0);
  initial_state.set_velocity(v0);
  initial_state.set_position(p0);
  initial_state.set_gyroscope_bias(bg0);
  initial_state.set_accelerometer_bias(ba0);

  // Initialize state covariance
  initial_state.set_rotation_covariance(0.03 * Eigen::Matrix3d::Identity());
  initial_state.set_velocity_covariance(0.01 * Eigen::Matrix3d::Identity());
  initial_state.set_position_covariance(0.00001 * Eigen::Matrix3d::Identity());
  initial_state.set_gyroscope_bias_covariance(0.0001
                                              * Eigen::Matrix3d::Identity());
  initial_state.set_accelerometer_bias_covariance(
      0.0025 * Eigen::Matrix3d::Identity());

  this->set_state(initial_state);
  std::cout << "Robot's state mean is initialized to: \n";
  std::cout << this->get_state().get_X() << std::endl;
  std::cout << "Robot's state covariance is initialized to: \n";
  std::cout << this->get_state().get_P() << std::endl;

  // Set enabled flag
  double t_prev = imu_packet_in.get_time();
  state_.set_time(t_prev);
  enabled_ = true;
}

/* ----------------------------------------------------------------------------
 * Copyright 2022, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

template<unsigned int JOINT_DIM, typename T>
JointStateMeasurement<JOINT_DIM, T>::JointStateMeasurement()
    : Measurement(JOINT_STATE) {
  joint_position_.setZero();
  joint_velocity_.setZero();
  joint_effort_.setZero();
}

template<unsigned int JOINT_DIM, typename T>
void JointStateMeasurement<JOINT_DIM, T>::set_joint_state(
    const Eigen::Matrix<T, JOINT_DIM, 1>& position,
    const Eigen::Matrix<T, JOINT_DIM, 1>& velocity,
    const Eigen::Matrix<T, JOINT_DIM, 1>& effort) {
  joint_position_ = position;
  joint_velocity_ = velocity;
  joint_effort_ = effort;
}

template<unsigned int JOINT_DIM, typename T>
Eigen::Matrix<T, JOINT_DIM, 1>
JointStateMeasurement<JOINT_DIM, T>::get_joint_pos() const {
  return joint_position_;
}

template<unsigned int JOINT_DIM, typename T>
Eigen::Matrix<T, JOINT_DIM, 1>
JointStateMeasurement<JOINT_DIM, T>::get_joint_vel() const {
  return joint_velocity_;
}

template<unsigned int JOINT_DIM, typename T>
Eigen::Matrix<T, JOINT_DIM, 1>
JointStateMeasurement<JOINT_DIM, T>::get_joint_effort() const {
  return joint_effort_;
}

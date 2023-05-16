/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   joint_state_impl.cpp
 *  @author Justin Yu
 *  @brief  Implementation for joint_state measurement
 *  @date   May 16, 2023
 **/

namespace measurement {
template<typename T>
JointStateMeasurement<T>::JointStateMeasurement() : Measurement(JOINT_STATE) {}

template<typename T>
void JointStateMeasurement<T>::set_joint_state(
    const Eigen::Matrix<T, Eigen::Dynamic, 1>& position,
    const Eigen::Matrix<T, Eigen::Dynamic, 1>& velocity,
    const Eigen::Matrix<T, Eigen::Dynamic, 1>& effort) {
  joint_position_ = position;
  joint_velocity_ = velocity;
  joint_effort_ = effort;
}

template<typename T>
void JointStateMeasurement<T>::set_encoders(
    const Eigen::Matrix<T, Eigen::Dynamic, 1>& position) {
  joint_position_ = position;
}

template<typename T>
Eigen::Matrix<T, Eigen::Dynamic, 1> JointStateMeasurement<T>::get_joint_pos()
    const {
  return joint_position_;
}

template<typename T>
Eigen::Matrix<T, Eigen::Dynamic, 1> JointStateMeasurement<T>::get_joint_vel()
    const {
  return joint_velocity_;
}

template<typename T>
Eigen::Matrix<T, Eigen::Dynamic, 1> JointStateMeasurement<T>::get_joint_effort()
    const {
  return joint_effort_;
}
}    // namespace measurement

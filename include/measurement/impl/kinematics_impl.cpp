/* ----------------------------------------------------------------------------
 * Copyright 2022, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

template<typename T>
KinematicsMeasurement<T>::KinematicsMeasurement() : Measurement(KINEMATICS) {
  position_.setZero();
  velocity_.setZero();
  effort_.setZero();
}

template<typename T>
void KinematicsMeasurement<T>::set_kin_state(
    const Eigen::Matrix<T, 3, 1>& pos, const Eigen::Matrix<T, 3, 1>& vel,
    const Eigen::Matrix<T, 3, 1>& eft) {
  position_ = pos;
  velocity_ = vel;
  effort_ = eft;
}

template<typename T>
Eigen::Matrix<T, 3, 1> KinematicsMeasurement<T>::get_kin_pos() const {
  return position_;
}

template<typename T>
Eigen::Matrix<T, 3, 1> KinematicsMeasurement<T>::get_kin_vel() const {
  return velocity_;
}

template<typename T>
Eigen::Matrix<T, 3, 1> KinematicsMeasurement<T>::get_kin_effort() const {
  return effort_;
}

template<typename T>
KinematicsMeasurement<T>::KinematicsMeasurement() : Measurement(KINEMATICS) {
  position_.setZero();
  velocity_.setZero();
  effort_.setZero();
}


template<typename T>
void KinematicsMeasurement<T>::set_kin_state(
    const Eigen::Matrix<T, 3, 1>& position,
    const Eigen::Matrix<T, 3, 1>& velocity,
    const Eigen::Matrix<T, 3, 1>& effort) {
  position_ = position;
  velocity_ = velocity;
  effort_ = effort;
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

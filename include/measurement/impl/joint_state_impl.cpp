template<unsigned int JOINT_DIM>
JointStateMeasurement<JOINT_DIM>::JointStateMeasurement()
    : Measurement(JOINT_STATE) {
  joint_position_.setZero();
  joint_velocity_.setZero();
  joint_effort_.setZero();
}

template<unsigned int JOINT_DIM>
Eigen::Matrix<double, JOINT_DIM, 1>
JointStateMeasurement<JOINT_DIM>::get_joint_pos() {
  return joint_position_;
}

template<unsigned int JOINT_DIM>
Eigen::Matrix<double, JOINT_DIM, 1>
JointStateMeasurement<JOINT_DIM>::get_joint_vel() {
  return joint_velocity_;
}

template<unsigned int JOINT_DIM>
Eigen::Matrix<double, JOINT_DIM, 1>
JointStateMeasurement<JOINT_DIM>::get_joint_effort() {
  return joint_effort_;
}

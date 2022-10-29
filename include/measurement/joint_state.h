#ifndef JOINT_STATE_H
#define JOINT_STATE_H

#include "measurement.h"

template<unsigned int JOINT_DIM>
class JointStateMeasurement : public Measurement {
 public:
  JointStateMeasurement();

  void set_joint_pos(Eigen::Matrix<double, JOINT_DIM, 1> pos);

  void set_joint_vel(Eigen::Matrix<double, JOINT_DIM, 1> vel);

  void set_joint_effort(Eigen::Matrix<double, JOINT_DIM, 1> effort);

  Eigen::Matrix<double, JOINT_DIM, 1> get_joint_pos();

  Eigen::Matrix<double, JOINT_DIM, 1> get_joint_vel();

  Eigen::Matrix<double, JOINT_DIM, 1> get_joint_effort();


 private:
  Eigen::Matrix<double, JOINT_DIM, 1> joint_position_;
  Eigen::Matrix<double, JOINT_DIM, 1> joint_velocity_;
  Eigen::Matrix<double, JOINT_DIM, 1> joint_effort_;
};
#include "measurement/impl/joint_state_impl.cpp"

#endif

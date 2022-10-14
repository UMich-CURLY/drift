#pragma once
#include <stdint.h>
#include <string>
#include "measurement.h"

// namespace cheetah_inekf_lcm {
class JointStateMeasurement : public Measurement {
 public:
  // Construct Encoder measurement
  JointStateMeasurement(unsigned int ENCODER_DIM) {
    type_ = JOINT_STATE;
    encoder_dim_ = ENCODER_DIM;
  }

  // void setKinematicsArray(const inekf_msgs::KinematicsArray& kinematics) {
  //     kin_arr = kinematics;
  // }

  // const inekf_msgs::KinematicsArray& getKinematicsArray() {
  //     return kin_arr;
  // }


  // std::vector<T> joint_position;
  // std::vector<T> joint_velocity;
  // std::vector<T> joint_effort;
 private:
  Eigen::VectorXd joint_position_;
  Eigen::VectorXd joint_velocity_;
  Eigen::VectorXd joint_effort_;
  unsigned int encoder_dim_;
  // inekf_msgs::KinematicsArray kin_arr;
};
//}    // namespace cheetah_inekf_lcm
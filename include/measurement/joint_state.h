#pragma once
#include <stdint.h>
#include <string>
#include "measurement.h"

// namespace cheetah_inekf_lcm {
class JointStateMeasurement : public Measurement {
 public:
  // Construct Encoder measurement
  JointStateMeasurement(unsigned int ENCODER_DIM);

 private:
  Eigen::VectorXd joint_position_;
  Eigen::VectorXd joint_velocity_;
  Eigen::VectorXd joint_effort_;
  unsigned int encoder_dim_;
};
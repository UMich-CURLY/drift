#include "kinematics/mini_cheetah_kinematics.h"

MiniCheetahKin::MiniCheetahKin() {
  jacobian_.resize(3, 12);
  jacobian_.setZero();
  contact_.resize(4, 1);
  contact_.setZero();
  encoder_position_.resize(12, 1);
  encoder_position_.setZero();
  body_to_foot_.resize(3, 4);
  body_to_foot_.setZero();
}

void MiniCheetahKin::compute_kinematics() {
  Eigen::Matrix<double, 12, 1> encoders = encoder_position_;

  Eigen::Matrix<double, 3, 1> p_FL = p_Body_to_FrontLeftFoot(encoders);
  Eigen::Matrix<double, 3, 1> p_FR = p_Body_to_FrontRightFoot(encoders);
  Eigen::Matrix<double, 3, 1> p_HL = p_Body_to_HindLeftFoot(encoders);
  Eigen::Matrix<double, 3, 1> p_HR = p_Body_to_HindRightFoot(encoders);
  Eigen::Matrix<double, 3, 12> JpFL = Jp_Body_to_FrontLeftFoot(encoders);
  Eigen::Matrix<double, 3, 12> JpFR = Jp_Body_to_FrontRightFoot(encoders);
  Eigen::Matrix<double, 3, 12> JpHL = Jp_Body_to_HindLeftFoot(encoders);
  Eigen::Matrix<double, 3, 12> JpHR = Jp_Body_to_HindRightFoot(encoders);

  body_to_foot_.col(0) = p_FR;
  body_to_foot_.col(1) = p_HR;
  body_to_foot_.col(2) = p_HL;
  body_to_foot_.col(3) = p_FL;
}

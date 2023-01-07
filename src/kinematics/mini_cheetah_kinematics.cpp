#include "kinematics/mini_cheetah_kinematics.h"

MiniCheetahKin::MiniCheetahKin() {}

void MiniCheetahKin::compute_kinematics() {
  Eigen::Matrix<double, 12, 1> encoders
      = this->get_joint_state().get_joint_pos();

  Eigen::Matrix<double, 3, 1> p_FL = p_Body_to_FrontLeftFoot(encoders);
  Eigen::Matrix<double, 3, 1> p_FR = p_Body_to_FrontRightFoot(encoders);
  Eigen::Matrix<double, 3, 1> p_HL = p_Body_to_HindLeftFoot(encoders);
  Eigen::Matrix<double, 3, 1> p_HR = p_Body_to_HindRightFoot(encoders);
  Eigen::Matrix<double, 3, 12> JpFL = Jp_Body_to_FrontLeftFoot(encoders);
  Eigen::Matrix<double, 3, 12> JpFR = Jp_Body_to_FrontRightFoot(encoders);
  Eigen::Matrix<double, 3, 12> JpHL = Jp_Body_to_HindLeftFoot(encoders);
  Eigen::Matrix<double, 3, 12> JpHR = Jp_Body_to_HindRightFoot(encoders);
}

void MiniCheetahKin::set_joint_state(
    const JointStateMeasurement<12, double>& js) {
  js_ = js;
}

void MiniCheetahKin::set_contact_state(const ContactMeasurement<4>& ct) {
  contact_ = ct;
}

JointStateMeasurement<12, double> MiniCheetahKin::get_joint_state() const {
  return js_;
}

ContactMeasurement<4> MiniCheetahKin::get_contact_state() const {
  return contact_;
}

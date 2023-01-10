#include "kinematics/mini_cheetah_kinematics.h"

MiniCheetahKin::MiniCheetahKin() {
  position_.setConstant(3, 4, 0);
  velocity_.setConstant(3, 4, 0);
  jacobian_.setConstant(3, 12, 0);
  contacts_.setConstant(4, 1, 0);
  encoders_.setConstant(12, 1, 0);
}

MiniCheetahKin::MiniCheetahKin(
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& encoders,
    const Eigen::Matrix<bool, Eigen::Dynamic, 1>& contacts)
    : legged_kinematics(encoders, contacts) {
  position_.setConstant(3, 4, 0);
  velocity_.setConstant(3, 4, 0);
  jacobian_.setConstant(3, 12, 0);
}

void MiniCheetahKin::compute_kinematics() {
  position_.col(FR) = p_Body_to_FrontRightFoot(encoders_);
  position_.col(FL) = p_Body_to_FrontLeftFoot(encoders_);
  position_.col(HL) = p_Body_to_HindLeftFoot(encoders_);
  position_.col(HR) = p_Body_to_HindRightFoot(encoders_);
  jacobian_.block(0, FR * 3, 3, 3)
      = Jp_Body_to_FrontRightFoot(encoders_).block(0, FR * 3, 3, 3);
  jacobian_.block(0, FL * 3, 3, 3)
      = Jp_Body_to_FrontLeftFoot(encoders_).block(0, FL * 3, 3, 3);
  jacobian_.block(0, HL * 3, 3, 3)
      = Jp_Body_to_HindLeftFoot(encoders_).block(0, HL * 3, 3, 3);
  jacobian_.block(0, HR * 3, 3, 3)
      = Jp_Body_to_HindRightFoot(encoders_).block(0, HR * 3, 3, 3);
}

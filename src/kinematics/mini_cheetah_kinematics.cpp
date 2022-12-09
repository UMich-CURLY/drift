#include "kinematics/mini_cheetah_kinematics.h"

void MiniCheetahKinematics::convertKinematics(CheetahState& state) {
  // Correct state based on kinematics measurements (probably in cheetah inekf
  // ros)
  Eigen::Matrix<double, 12, 1> encoders = state.getEncoderPositions();

  Eigen::Matrix4d H_FL = H_Body_to_FrontLeftFoot(encoders);
  Eigen::Matrix4d H_FR = H_Body_to_FrontRightFoot(encoders);
  Eigen::Matrix4d H_HL = H_Body_to_HindLeftFoot(encoders);
  Eigen::Matrix4d H_HR = H_Body_to_HindRightFoot(encoders);
  Eigen::Matrix<double, 3, 12> JpFL = Jp_Body_to_FrontLeftFoot(encoders);
  Eigen::Matrix<double, 3, 12> JpFR = Jp_Body_to_FrontRightFoot(encoders);
  Eigen::Matrix<double, 3, 12> JpHL = Jp_Body_to_HindLeftFoot(encoders);
  Eigen::Matrix<double, 3, 12> JpHR = Jp_Body_to_HindRightFoot(encoders);
  Eigen::Matrix<double, 6, 6> covFL = Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::Matrix<double, 6, 6> covFR = Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::Matrix<double, 6, 6> covHL = Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::Matrix<double, 6, 6> covHR = Eigen::Matrix<double, 6, 6>::Identity();
  covFL.block<3, 3>(3, 3)
      = JpFL * encoder_cov_ * JpFL.transpose() + prior_kinematics_cov_;
  covFR.block<3, 3>(3, 3)
      = JpFR * encoder_cov_ * JpFR.transpose() + prior_kinematics_cov_;
  covHL.block<3, 3>(3, 3)
      = JpHL * encoder_cov_ * JpHL.transpose() + prior_kinematics_cov_;
  covHR.block<3, 3>(3, 3)
      = JpHR * encoder_cov_ * JpHR.transpose() + prior_kinematics_cov_;
  inekf::Kinematics rightFrontFoot(0, H_FR, covFR);
  inekf::Kinematics leftFrontFoot(1, H_FL, covFL);
  inekf::Kinematics rightHindFoot(2, H_HR, covHR);
  inekf::Kinematics leftHindFoot(3, H_HL, covHL);
  inekf::vectorKinematics kinematics;
  kinematics.push_back(rightFrontFoot);
  kinematics.push_back(leftFrontFoot);
  kinematics.push_back(rightHindFoot);
  kinematics.push_back(leftHindFoot);

  filter_.CorrectKinematics(kinematics);
  if (estimator_debug_enabled_) {
    auto position = filter_.getState().getPosition();
    printf("Kinematics correction complete x: %0.6f y: %0.6f z: %0.6f\n",
           position[0], position[1], position[2]);
  }
}
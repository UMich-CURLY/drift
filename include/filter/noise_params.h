/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   NoiseParams.h
 *  @author Ross Hartley
 *  @brief  Header file for Invariant EKF noise parameter class
 *  @date   November 25, 2022
 **/
#ifndef INEKF_NOISEPARAMS_H
#define INEKF_NOISEPARAMS_H
#include <Eigen/Dense>
#include <iostream>
#include <unordered_map>

class NoiseParams {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  NoiseParams();

  void set_gyroscope_noise(double std);
  void set_gyroscope_noise(const Eigen::Vector3d& std);
  void set_gyroscope_noise(const Eigen::Matrix3d& cov);

  void set_accelerometer_noise(double std);
  void set_accelerometer_noise(const Eigen::Vector3d& std);
  void set_accelerometer_noise(const Eigen::Matrix3d& cov);

  void set_gyroscope_bias_noise(double std);
  void set_gyroscope_bias_noise(const Eigen::Vector3d& std);
  void set_gyroscope_bias_noise(const Eigen::Matrix3d& cov);

  void set_accelerometer_bias_noise(double std);
  void set_accelerometer_bias_noise(const Eigen::Vector3d& std);
  void set_accelerometer_bias_noise(const Eigen::Matrix3d& cov);

  void setAugmentNoise(double std);
  void setAugmentNoise(const Eigen::Vector3d& std);
  void setAugmentNoise(const Eigen::Matrix3d& cov);

  void set_augment_noise(const std::string& aug_type, const double std);
  void set_augment_noise(const std::string& aug_type,
                         const Eigen::Vector3d& std);
  void set_augment_noise(const std::string& aug_type,
                         const Eigen::Matrix3d& std);

  const Eigen::Matrix3d get_gyroscope_cov() const;
  const Eigen::Matrix3d get_accelerometer_cov() const;
  const Eigen::Matrix3d get_gyroscope_bias_cov() const;
  const Eigen::Matrix3d get_accelerometer_bias_cov() const;
  const Eigen::Matrix3d get_augment_cov(const std::string& aug_type) const;

  friend std::ostream& operator<<(std::ostream& os, const NoiseParams& p);

 private:
  Eigen::Matrix3d Qg_;
  Eigen::Matrix3d Qa_;
  Eigen::Matrix3d Qbg_;
  Eigen::Matrix3d Qba_;
  Eigen::Matrix3d Ql_;
  Eigen::Matrix3d Qc_;

  std::unordered_map<std::string, Eigen::Matrix3d> aug2cov_;
};

#endif
/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   NoiseParams.cpp
 *  @author Ross Hartley
 *  @brief  Source file for Invariant EKF noise parameter class
 *  @date   November 25, 2022
 **/

#include "filter/noise_params.h"

using namespace std;

// ------------ NoiseParams -------------
// Default Constructor
NoiseParams::NoiseParams() {
  set_gyroscope_noise(0.01);
  set_accelerometer_noise(0.1);
  set_gyroscope_bias_noise(0.00001);
  set_accelerometer_bias_noise(0.0001);
}

void NoiseParams::set_gyroscope_noise(double std) {
  Qg_ = std * std * Eigen::Matrix3d::Identity();
}
void NoiseParams::set_gyroscope_noise(const Eigen::Vector3d& std) {
  Qg_ << std(0) * std(0), 0, 0, 0, std(1) * std(1), 0, 0, 0, std(2) * std(2);
}
void NoiseParams::set_gyroscope_noise(const Eigen::Matrix3d& cov) { Qg_ = cov; }

void NoiseParams::set_accelerometer_noise(double std) {
  Qa_ = std * std * Eigen::Matrix3d::Identity();
}
void NoiseParams::set_accelerometer_noise(const Eigen::Vector3d& std) {
  Qa_ << std(0) * std(0), 0, 0, 0, std(1) * std(1), 0, 0, 0, std(2) * std(2);
}
void NoiseParams::set_accelerometer_noise(const Eigen::Matrix3d& cov) {
  Qa_ = cov;
}

void NoiseParams::set_gyroscope_bias_noise(double std) {
  Qbg_ = std * std * Eigen::Matrix3d::Identity();
}
void NoiseParams::set_gyroscope_bias_noise(const Eigen::Vector3d& std) {
  Qbg_ << std(0) * std(0), 0, 0, 0, std(1) * std(1), 0, 0, 0, std(2) * std(2);
}
void NoiseParams::set_gyroscope_bias_noise(const Eigen::Matrix3d& cov) {
  Qbg_ = cov;
}

void NoiseParams::set_accelerometer_bias_noise(double std) {
  Qba_ = std * std * Eigen::Matrix3d::Identity();
}
void NoiseParams::set_accelerometer_bias_noise(const Eigen::Vector3d& std) {
  Qba_ << std(0) * std(0), 0, 0, 0, std(1) * std(1), 0, 0, 0, std(2) * std(2);
}
void NoiseParams::set_accelerometer_bias_noise(const Eigen::Matrix3d& cov) {
  Qba_ = cov;
}


void NoiseParams::set_augment_noise(const std::string& aug_type,
                                    const double std) {
  aug2cov_[aug_type] << std * std * Eigen::Matrix3d::Identity();
}
void NoiseParams::set_augment_noise(const std::string& aug_type,
                                    const Eigen::Vector3d& std) {
  aug2cov_[aug_type] << std(0) * std(0), 0, 0, 0, std(1) * std(1), 0, 0, 0,
      std(2) * std(2);
}
void NoiseParams::set_augment_noise(const std::string& aug_type,
                                    const Eigen::Matrix3d& cov) {
  aug2cov_[aug_type] << cov;
}


const Eigen::Matrix3d NoiseParams::get_gyroscope_cov() const { return Qg_; }
const Eigen::Matrix3d NoiseParams::get_accelerometer_cov() const { return Qa_; }
const Eigen::Matrix3d NoiseParams::get_gyroscope_bias_cov() const {
  return Qbg_;
}
const Eigen::Matrix3d NoiseParams::get_accelerometer_bias_cov() const {
  return Qba_;
}
const Eigen::Matrix3d NoiseParams::get_augment_cov(
    const std::string& aug_type) const {
  return aug2cov_.at(aug_type);
}


std::ostream& operator<<(std::ostream& os, const NoiseParams& p) {
  os << "--------- Noise Params -------------" << endl;
  os << "Gyroscope Covariance:\n" << p.Qg_ << endl;
  os << "Accelerometer Covariance:\n" << p.Qa_ << endl;
  os << "Gyroscope Bias Covariance:\n" << p.Qbg_ << endl;
  os << "Accelerometer Bias Covariance:\n" << p.Qba_ << endl;
  os << "Contact Covariance:\n" << p.Qc_ << endl;
  os << "Augment state Covariance:\n" << p.Qc_ << endl;
  os << "-----------------------------------" << endl;
  return os;
}

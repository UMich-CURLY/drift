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
  setGyroscopeNoise(0.01);
  setAccelerometerNoise(0.1);
  setGyroscopeBiasNoise(0.00001);
  setAccelerometerBiasNoise(0.0001);
  setContactNoise(0.1);
  setAugmentNoise(0.1);
}

void NoiseParams::setGyroscopeNoise(double std) {
  Qg_ = std * std * Eigen::Matrix3d::Identity();
}
void NoiseParams::setGyroscopeNoise(const Eigen::Vector3d& std) {
  Qg_ << std(0) * std(0), 0, 0, 0, std(1) * std(1), 0, 0, 0, std(2) * std(2);
}
void NoiseParams::setGyroscopeNoise(const Eigen::Matrix3d& cov) { Qg_ = cov; }

void NoiseParams::setAccelerometerNoise(double std) {
  Qa_ = std * std * Eigen::Matrix3d::Identity();
}
void NoiseParams::setAccelerometerNoise(const Eigen::Vector3d& std) {
  Qa_ << std(0) * std(0), 0, 0, 0, std(1) * std(1), 0, 0, 0, std(2) * std(2);
}
void NoiseParams::setAccelerometerNoise(const Eigen::Matrix3d& cov) {
  Qa_ = cov;
}

void NoiseParams::setGyroscopeBiasNoise(double std) {
  Qbg_ = std * std * Eigen::Matrix3d::Identity();
}
void NoiseParams::setGyroscopeBiasNoise(const Eigen::Vector3d& std) {
  Qbg_ << std(0) * std(0), 0, 0, 0, std(1) * std(1), 0, 0, 0, std(2) * std(2);
}
void NoiseParams::setGyroscopeBiasNoise(const Eigen::Matrix3d& cov) {
  Qbg_ = cov;
}

void NoiseParams::setAccelerometerBiasNoise(double std) {
  Qba_ = std * std * Eigen::Matrix3d::Identity();
}
void NoiseParams::setAccelerometerBiasNoise(const Eigen::Vector3d& std) {
  Qba_ << std(0) * std(0), 0, 0, 0, std(1) * std(1), 0, 0, 0, std(2) * std(2);
}
void NoiseParams::setAccelerometerBiasNoise(const Eigen::Matrix3d& cov) {
  Qba_ = cov;
}

void NoiseParams::setContactNoise(double std) {
  Qc_ = std * std * Eigen::Matrix3d::Identity();
}
void NoiseParams::setContactNoise(const Eigen::Vector3d& std) {
  Qc_ << std(0) * std(0), 0, 0, 0, std(1) * std(1), 0, 0, 0, std(2) * std(2);
}
void NoiseParams::setContactNoise(const Eigen::Matrix3d& cov) { Qc_ = cov; }

void NoiseParams::setAugmentNoise(double std) {
  Qc_ = std * std * Eigen::Matrix3d::Identity();
}
void NoiseParams::setAugmentNoise(const Eigen::Vector3d& std) {
  Qc_ << std(0) * std(0), 0, 0, 0, std(1) * std(1), 0, 0, 0, std(2) * std(2);
}
void NoiseParams::setAugmentNoise(const Eigen::Matrix3d& cov) { Qc_ = cov; }

const Eigen::Matrix3d NoiseParams::getGyroscopeCov() const { return Qg_; }
const Eigen::Matrix3d NoiseParams::getAccelerometerCov() const { return Qa_; }
const Eigen::Matrix3d NoiseParams::getGyroscopeBiasCov() const { return Qbg_; }
const Eigen::Matrix3d NoiseParams::getAccelerometerBiasCov() const {
  return Qba_;
}
const Eigen::Matrix3d NoiseParams::getContactCov() const { return Qc_; }
const Eigen::Matrix3d NoiseParams::getAugmentCov() const { return Qc_; }


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

/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   odom.h
 *  @author Tingjun Li
 *  @brief  Header file for robot imu estimate measurement
 *  @date   May 27, 2023
 **/

#ifndef MEASUREMENT_ODOM_H
#define MEASUREMENT_ODOM_H

#include "measurement.h"

namespace measurement {
class OdomMeasurement : public Measurement {
 public:
  // Construct Encoder measurement
  OdomMeasurement() {
    type_ = ODOM;
    translation_ = Eigen::Vector3d::Zero();
    rotation_ = Eigen::Matrix3d::Identity();
    transformation_ = Eigen::Matrix4d::Identity();
  }

  OdomMeasurement(const Eigen::Vector3d& translation,
                  const Eigen::Quaternion<double>& quaternion,
                  const uint64_t seq_in, const double time_stamp_in,
                  const std::string frame_id_in) {
    type_ = ODOM;

    translation_ = Eigen::Vector3d::Zero();

    rotation_ = Eigen::Matrix3d::Identity();

    transformation_ = Eigen::Matrix4d::Identity();

    set_translation(translation);    // [x, y, z]
    set_rotation(quaternion);        // [x, y, z]
    set_header(seq_in, time_stamp_in, frame_id_in);

    set_transformation();
  }


  void set_translation(const Eigen::Vector3d& position_in) {
    translation_(0) = position_in(0);
    translation_(1) = position_in(1);
    translation_(2) = position_in(2);
  }

  void set_rotation(const Eigen::Quaternion<double>& orientation_quat) {
    rotation_ = orientation_quat.toRotationMatrix();
  }

  void set_transformation() {
    transformation_.block<3, 3>(0, 0) = rotation_;
    transformation_.block<3, 1>(0, 3) = translation_;
  }

  inline const Eigen::Matrix4d& get_transformation() const {
    return transformation_;
  }


 private:
  Eigen::Vector3d translation_;
  Eigen::Matrix3d rotation_;
  Eigen::Matrix4d transformation_;
};

}    // namespace measurement

#endif    // MEASUREMENT_ODOM_H
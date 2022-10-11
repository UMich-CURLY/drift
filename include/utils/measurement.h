/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   measurement.h
 *  @author Ross Hartley
 *  @brief  Header file for Measurement class
 *  @date   September 27, 2018
 **/

#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include <iostream>
#include <string>

#include <Eigen/Dense>

enum MeasurementType { EMPTY, IMU, KINEMATICS, CONTACT, JOINT_STATE };

class Measurement {
  struct MeasurementHeader {
    uint64_t seq;
    double stamp;
    std::string frame_id;
  };

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Measurement();
  virtual ~Measurement() = default;

  MeasurementHeader header;

  // ======================================================================
  /**
   * @brief Get the timestamp value for the measurement.
   *
   * @return double: The timestamp.
   */
  // ======================================================================
  double get_time();

  // ======================================================================
  /**
   * @brief Get the measurement type.
   *
   * @return MeasurementType: The measurement type.
   */
  // ======================================================================
  MeasurementType get_type();

  friend std::ostream& operator<<(std::ostream& os, const Measurement& m);

 protected:
  MeasurementType type_;
};

struct MeasurementCompare {
  bool operator()(Measurement& lhs, Measurement& rhs) const {
    return lhs.get_time() > rhs.get_time();
  }
};

#endif

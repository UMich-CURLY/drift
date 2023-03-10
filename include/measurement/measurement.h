/* ----------------------------------------------------------------------------
 * Copyright 2022, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   measurement.h
 *  @author Justin Yu
 *  @brief  Header file for measurement base class
 *  @date   Nov 16, 2022
 **/

#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include <iostream>
#include <string>

#include <Eigen/Dense>

enum MeasurementType {
  EMPTY,
  IMU,
  LEGGED_KINEMATICS,
  VELOCITY,
  JOINT_STATE,
  CONTACT,
  NAVSAT
};

/**
 * @class Measurement
 *
 * Base class for robot-state instantaneous measurement.
 * contact state.
 */
class Measurement {
  struct MeasurementHeader {
    uint64_t seq;
    double stamp;
    std::string frame_id;
  };

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /**
   * @brief Default constructor.
   */
  Measurement();

  /**
   * @brief Constructs an initialized Measurement with given type.
   */
  Measurement(MeasurementType type);
  /**
   * @brief Default Destructor.
   */
  virtual ~Measurement() = default;

  MeasurementHeader header;

  /**
   * @brief Set the timestamp value for the measurement.
   *
   * @param[in] stamp: The timestamp.
   */
  void set_time(double stamp);

  /**
   * @brief Set the header value for the measurement.
   *
   * @param[in] header: The MeasurementHeader POD.
   */
  void set_header(const MeasurementHeader& header);

  /**
   * @brief Set the header value for the measurement.
   *
   * @param[in] seq_in: The sequence number
   * @param[in] time_stamp_in: The time stamp of this measurement
   * @param[in] frame_id_in: The frame id
   */
  void set_header(const uint64_t seq_in, const double time_stamp_in,
                  const std::string frame_id_in);

  /**
   * @brief Get the timestamp value for the measurement.
   *
   * @return Measurement timestamp.
   */
  double get_time() const;

  /**
   * @brief Get the measurement type.
   *
   * @return Measurement type.
   */
  MeasurementType get_type() const;

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

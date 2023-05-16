/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   measurement.cpp
 *  @author Justin Yu
 *  @brief  Source file for measurement base class
 *  @date   May 16, 2023
 **/

#include "drift/measurement/measurement.h"

using namespace std;

namespace measurement {

Measurement::Measurement() : type_(EMPTY) { header.stamp = 0; }

Measurement::Measurement(MeasurementType type) : type_(type) {
  header.stamp = 0;
}
void Measurement::set_time(double t) { header.stamp = t; }

void Measurement::set_header(const MeasurementHeader& h) { header = h; }

void Measurement::set_header(const uint64_t seq_in, const double time_stamp_in,
                             const std::string frame_id_in) {
  header.seq = seq_in;
  header.stamp = time_stamp_in;
  header.frame_id = frame_id_in;
};

double Measurement::get_time() const { return header.stamp; }

MeasurementType Measurement::get_type() const { return type_; }

ostream& operator<<(ostream& os, const Measurement& m) {
  string type_str;
  switch (m.type_) {
    case IMU:
      type_str = "IMU";
      break;
    case LEGGED_KINEMATICS:
      type_str = "LEGGED_KINEMATICS";
      break;
    case VELOCITY:
      type_str = "VELOCITY";
      break;
    case JOINT_STATE:
      type_str = "JOINT_STATE";
      break;
    case CONTACT:
      type_str = "CONTACT";
      break;
    default:
      type_str = "UNKNOWN";
  }
  os << "Measurement type: " << type_str << endl;
  return os;
}

}    // namespace measurement

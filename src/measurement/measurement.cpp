/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   measurement.cpp
 *  @author Ross Hartley
 *  @brief  Source file for Measurement class
 *  @date   September 27, 2018
 **/

#include "measurement/measurement.h"

using namespace std;

// using namespace inekf;

Measurement::Measurement() {
  header.stamp = 0;
  type_ = EMPTY;
}

double Measurement::get_time() { return header.stamp; }

MeasurementType Measurement::get_type() { return type_; }

ostream& operator<<(ostream& os, const Measurement& m) {
  string type_str;
  switch (m.type_) {
    case IMU:
      type_str = "IMU";
      break;
    default:
      type_str = "Unknown";
  }
  os << "Measurement type: " << type_str << endl;
  return os;
}

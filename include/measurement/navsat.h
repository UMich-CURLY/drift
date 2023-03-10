/* ----------------------------------------------------------------------------
 * Copyright 2022, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   navsat.h
 *  @author Justin Yu
 *  @brief  Header file for robot nav sat (gps) position estimate measurement
 *  @date   Mar 10, 2023
 **/

#ifndef NAVSAT_H
#define NAVSAT_H

#include <math.h>
#include "measurement.h"
/**
 * @class GeoSpaceMeasurement
 *
 * Derived measurement class containing NavSat information (longitude, latitude,
 * altitude)
 */
template<typename T>
class NavSatMeasurement : public Measurement {
  struct Ellipsoid {
    double semimajor_axis;
    double semiminor_axis;
  };

 public:
  /**
   * @brief Default constructor.
   */
  NavSatMeasurement();

  /**
   * @brief Overload constructor.
   */
  NavSatMeasurement(bool deg_);

  void set_geodetic(T lat, T lon, T alt);

  Eigen::Matrix<T, 3, 1> get_geodetic();

  Eigen::Matrix<T, 3, 1> get_enu(T lat0, T lon0, T alt0);

 private:
  Eigen::Matrix<T, 3, 1> geodetic2ecef(T lat, T lon, T alt, , Ellipsoid ell);
  Eigen::Matrix<T, 3, 1> uvw2enu(T u, T v, T w, T lat0, T lon0);
  Eigen::Matrix<T, 3, 1> geodetic_;
  bool deg_;
};
#include "measurement/impl/navsat_impl.cpp"

#endif    // NAVSAT_H

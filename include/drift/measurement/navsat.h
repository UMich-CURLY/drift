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

namespace measurement {
/**
 * @class VelocityMeasurement
 *
 * @brief measurement class containing robot-frame
 * velocity information.
 */
/**
 * @class NavSatMeasurement
 *
 * Derived measurement class containing NavSatFix and ENU information transform.
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
   * @brief Set the geodetic measurement coefficients.
   * @param[in] lat: latitude (degrees). Positive is north of equator; negative
   * is south.
   * @param[in] lon: longitude (degrees). Positive is east of prime
   * meridian; negative is west.
   * @param[in] alt: altitude (m). Positive is above the ellipsoid definition.
   */
  void set_navsatfix(T lat, T lon, T alt);

  /**
   * @brief Get the geodetic measurement coefficients.
   *
   * @return the latitude, longitude, altitude (deg, deg, m)
   */
  Eigen::Matrix<T, 3, 1> get_navsatfix() const;

  /**
   * @brief Get the ENU (East, North, Up) measurement coefficients.
   *
   * @param[in] lat0: initial latitude (degrees).
   * @param[in] lon0: initial longitude (degrees).
   * @param[in] alt0: initial altidude (degrees).
   * @return the east, north, up (m, m, m) displacement relative to initial
   * state.
   */
  Eigen::Matrix<T, 3, 1> get_enu(T lat0, T lon0, T alt0) const;

 private:
  Eigen::Matrix<T, 3, 1> navsatfix_;
  constexpr static Ellipsoid WGS84_ = {6378137.0, 6356752.314245};
  Eigen::Matrix<T, 3, 1> geodetic2ecef(
      T lat, T lon, T alt,
      Ellipsoid ell = WGS84_) const;    // uses WGS 84 definition
  Eigen::Matrix<T, 3, 1> uvw2enu(T u, T v, T w, T lat0, T lon0) const;
};
}    // namespace measurement
#include "drift/measurement/impl/navsat_impl.cpp"

#endif    // NAVSAT_H

/* ----------------------------------------------------------------------------
 * Copyright 2022, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

namespace measurement {
template<typename T>
NavSatMeasurement<T>::NavSatMeasurement() : Measurement(NAVSAT) {}

template<typename T>
void NavSatMeasurement<T>::set_navsatfix(T lat, T lon, T alt) {
  navsatfix_(0) = lat;
  navsatfix_(1) = lon;
  navsatfix_(2) = alt;
}

template<typename T>
Eigen::Matrix<T, 3, 1> NavSatMeasurement<T>::get_navsatfix() const {
  return navsatfix_;
}

template<typename T>
Eigen::Matrix<T, 3, 1> NavSatMeasurement<T>::get_enu(T lat0, T lon0,
                                                     T alt0) const {
  Eigen::Matrix<T, 3, 1> ecef;
  ecef = geodetic2ecef(navsatfix_(0), navsatfix_(1), navsatfix_(2));
  T x1 = ecef(0);
  T y1 = ecef(1);
  T z1 = ecef(2);
  Eigen::Matrix<T, 3, 1> ecef0;
  ecef0 = geodetic2ecef(lat0, lon0, alt0);
  T x2 = ecef0(0);
  T y2 = ecef0(1);
  T z2 = ecef0(2);

  return uvw2enu(x1 - x2, y1 - y2, z1 - z2, lat0, lon0);
}

template<typename T>
Eigen::Matrix<T, 3, 1> NavSatMeasurement<T>::geodetic2ecef(
    T lat, T lon, T alt, NavSatMeasurement<T>::Ellipsoid ell) const {
  lat = lat * M_PI / 180.0;
  lon = lon * M_PI / 180.0;

  double N
      = ell.semimajor_axis * ell.semimajor_axis
        / sqrt(ell.semimajor_axis * ell.semimajor_axis * cos(lat) * cos(lat)
               + ell.semiminor_axis * ell.semiminor_axis * sin(lat) * sin(lat));
  T x = (N + alt) * cos(lat) * cos(lon);
  T y = (N + alt) * cos(lat) * sin(lon);
  T z = (N * ell.semiminor_axis * ell.semiminor_axis
             / (ell.semimajor_axis * ell.semimajor_axis)
         + alt)
        * sin(lat);


  Eigen::Matrix<T, 3, 1> ecef;
  ecef << x, y, z;

  return ecef;
}

template<typename T>
Eigen::Matrix<T, 3, 1> NavSatMeasurement<T>::uvw2enu(T u, T v, T w, T lat0,
                                                     T lon0) const {
  lat0 = lat0 * M_PI / 180.0;
  lon0 = lon0 * M_PI / 180.0;

  T t = cos(lon0) * u + sin(lon0) * v;
  T east = -sin(lon0) * u + cos(lon0) * v;
  T north = -sin(lat0) * t + cos(lat0) * w;
  T up = cos(lat0) * t + sin(lat0) * w;

  Eigen::Matrix<T, 3, 1> enu;
  enu << east, north, up;

  return enu;
}
}    // namespace measurement
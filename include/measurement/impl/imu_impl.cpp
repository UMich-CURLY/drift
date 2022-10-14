#include "measurement/imu.h"
template<typename T>
ImuMeasurement<T>::ImuMeasurement() {
  type_ = IMU;
}

template<typename T>
Eigen::Matrix3d ImuMeasurement<T>::get_rotation() {
  return R_;
}

template<typename T>
void ImuMeasurement<T>::set_rotation() {
  Eigen::Quaternion<double> q(get_orientation().w, get_orientation().x,
                              get_orientation().y, get_orientation().z);
  R_ = q.toRotationMatrix();
}

template<typename T>
ImuOrientation<T> ImuMeasurement<T>::get_orientation() {
  return orientation_;
}

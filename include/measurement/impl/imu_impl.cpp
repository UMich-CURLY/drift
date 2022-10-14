
template<typename T>
ImuMeasurement<T>::ImuMeasurement() {
  type_ = IMU;
}

template<typename T>
Eigen::Matrix3d ImuMeasurement<T>::toRotationMatrix() {
  return R_;
}

template<typename T>
void ImuMeasurement<T>::set_rotation() {
  Eigen::Quaternion<double> q(orientation_w(), orientation_x(), orientation_y(),
                              orientation_z());
  R_ = q.toRotationMatrix();
}

template<typename T>
ImuOrientation<T> ImuMeasurement<T>::orientation_w() {
  return orientation_.w;
}

template<typename T>
ImuOrientation<T> ImuMeasurement<T>::orientation_x() {
  return orientation_.x;
}

template<typename T>
ImuOrientation<T> ImuMeasurement<T>::orientation_y() {
  return orientation_.y;
}

template<typename T>
ImuOrientation<T> ImuMeasurement<T>::orientation_z() {
  return orientation_.z;
}

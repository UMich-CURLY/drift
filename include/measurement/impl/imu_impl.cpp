
template<typename T>
ImuMeasurement<T>::ImuMeasurement() {
  type_ = IMU;
}

template<typename T>
Eigen::Matrix3d ImuMeasurement<T>::get_rotation_matrix() {
  set_rotation();
  return R_;
}

template<typename T>
void ImuMeasurement<T>::set_rotation() {
  Eigen::Quaternion<double> q(get_quaternion_w(), get_quaternion_x(),
                              get_quaternion_y(), get_quaternion_z());
  R_ = q.toRotationMatrix();
}

template<typename T>
T ImuMeasurement<T>::get_quaternion_w() {
  return orientation_.w;
}

template<typename T>
T ImuMeasurement<T>::get_quaternion_x() {
  return orientation_.x;
}

template<typename T>
T ImuMeasurement<T>::get_quaternion_y() {
  return orientation_.y;
}

template<typename T>
T ImuMeasurement<T>::get_quaternion_z() {
  return orientation_.z;
}

template<typename T>
void ImuMeasurement<T>::set_quaternion(T x, T y, T z, T w) {
  orientation_.x = x;
  orientation_.y = y;
  orientation_.z = z;
  orientation_.w = w;
}

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
  return quaternion_.w;
}

template<typename T>
T ImuMeasurement<T>::get_quaternion_x() {
  return quaternion_.x;
}

template<typename T>
T ImuMeasurement<T>::get_quaternion_y() {
  return quaternion_.y;
}

template<typename T>
T ImuMeasurement<T>::get_quaternion_z() {
  return quaternion_.z;
}

template<typename T>
void ImuMeasurement<T>::set_quaternion(T x, T y, T z, T w) {
  quaternion_.x = x;
  quaternion_.y = y;
  quaternion_.z = z;
  quaternion_.w = w;
}

template<typename T>
ImuMeasurement<T>::ImuMeasurement()
    : Measurement(IMU),
      R_(Eigen::Matrix3d::Identity()),
      quaternion_{1, 0, 0, 0},
      angular_velocity_{0, 0, 0},
      linear_acceleration_{0, 0, 0} {
  type_ = IMU;
}

template<typename T>
void ImuMeasurement<T>::set_quaternion(T x, T y, T z, T w) {
  quat_inv(x, y, z, w);

  quaternion_.x = x;
  quaternion_.y = y;
  quaternion_.z = z;
  quaternion_.w = w;
}

template<typename T>
Eigen::Matrix3d ImuMeasurement<T>::get_rotation_matrix() {
  set_rotation();
  return R_;
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


// Private Functions

template<typename T>
void ImuMeasurement<T>::set_rotation() {
  Eigen::Quaternion<double> q(get_quaternion_w(), get_quaternion_x(),
                              get_quaternion_y(), get_quaternion_z());
  R_ = q.toRotationMatrix();
}


template<typename T>
void ImuMeasurement<T>::quat_inv(T x, T y, T z, T w) {
  double tol = 1e-5;
  T q_mag = std::sqrt(x * x + y * y + z * z + w * w);
  // std::cout << std::abs(q_mag - 1);
  if (std::abs(q_mag - 1) >= tol) {
    throw std::invalid_argument("Quaternion arguments must be normalized");
  }
}
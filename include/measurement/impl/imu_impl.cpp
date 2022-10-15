template<typename T>
ImuMeasurement<T>::ImuMeasurement()
    : Measurement(IMU),
      R_(Eigen::Matrix3d::Identity()),
      quaternion_{1, 0, 0, 0},
      angular_velocity_{0, 0, 0},
      linear_acceleration_{0, 0, 0} {}

template<typename T>
void ImuMeasurement<T>::set_quaternion(T w, T x, T y, T z) {
  quat_inv(w, x, y, z);

  quaternion_.w = w;
  quaternion_.x = x;
  quaternion_.y = y;
  quaternion_.z = z;
}

template<typename T>
void ImuMeasurement<T>::set_ang_vel(T x, T y, T z) {
  angular_velocity_.x = x;
  angular_velocity_.y = y;
  angular_velocity_.z = z;
}

template<typename T>
void ImuMeasurement<T>::set_lin_acc(T x, T y, T z) {
  linear_acceleration_.x = x;
  linear_acceleration_.y = y;
  linear_acceleration_.z = z;
}

template<typename T>
Eigen::Matrix3d ImuMeasurement<T>::get_rotation_matrix() {
  set_rotation();
  return R_;
}

template<typename T>
ImuQuaternion<T> ImuMeasurement<T>::get_quaternion() {
  return quaternion_;
}

template<typename T>
ImuAngularVelocity<T> ImuMeasurement<T>::get_ang_vel() {
  return angular_velocity_;
}

template<typename T>
ImuLinearAcceleration<T> ImuMeasurement<T>::get_lin_acc() {
  return linear_acceleration_;
}

// Private Functions

template<typename T>
void ImuMeasurement<T>::set_rotation() {
  Eigen::Quaternion<double> q(get_quaternion().w, get_quaternion().x,
                              get_quaternion().y, get_quaternion().z);
  R_ = q.toRotationMatrix();
}

template<typename T>
void ImuMeasurement<T>::quat_inv(T w, T x, T y, T z) {
  double tol = 1e-5;
  T q_mag = w * w + x * x + y * y + z * z;
  // std::cout << std::abs(q_mag - 1);
  if (std::abs(q_mag - 1) >= tol) {
    throw std::invalid_argument("Quaternion arguments must be normalized");
  }
}
template<typename T>
VelocityMeasurement<T>::VelocityMeasurement()
    : Measurement(VELOCITY), vel_{0, 0, 0} {}

template<typename T>
void VelocityMeasurement<T>::set_velocity(T vx, T vy, T vz) {
  vel_inv(vx, vy, vz);
  vel_.x = vx;
  vel_.y = vy;
  vel_.z = vz;
}

template<typename T>
Velocity<T> VelocityMeasurement<T>::get_velocity() {
  return vel_;
}

template<typename T>
double VelocityMeasurement<T>::get_vel_mag() {
  return std::sqrt(vel_.x * vel_.x + vel_.y * vel_.y + vel_.z * vel_.z);
}

template<typename T>
Eigen::Matrix<double, 3, 1> VelocityMeasurement<T>::get_vel_unit_vec() {
  Eigen::Matrix<double, 3, 1> uv;
  uv << vel_.x / get_vel_mag(), vel_.y / get_vel_mag(), vel_.z / get_vel_mag();
  return uv;
}

template<typename T>
void VelocityMeasurement<T>::vel_inv(T vx, T vy, T vz) {
  int c = 299792458;
  if (std::abs(get_vel_mag()) >= c) {
    throw std::invalid_argument("Velocity argument is FTL!");
  }
}
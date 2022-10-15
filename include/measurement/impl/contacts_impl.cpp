template<typename T>
ContactsMeasurement<T>::ContactsMeasurement() : Measurement(CONTACT) {}

template<typename T>
void ContactsMeasurement::setContacts(
    const Eigen::Matrix<bool, Eigen::Dynamic, 1>& contacts) {
  contacts_ = contacts;
}

template<typename T>
Eigen::Matrix<bool, Eigen::Dynamic, 1> ContactsMeasurement::getContacts() {
  return contacts_;
}

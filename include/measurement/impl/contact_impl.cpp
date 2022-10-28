template<int NUMCONTACTS>
ContactMeasurement<NUMCONTACTS>::ContactMeasurement() : Measurement(CONTACT) {
  contacts_.setZero();
}

template<int NUMCONTACTS>
void ContactMeasurement<NUMCONTACTS>::set_contact(
    const Eigen::Matrix<bool, NUMCONTACTS, 1>& contacts) {
  contacts_ = contacts;
}

template<int NUMCONTACTS>
Eigen::Matrix<bool, NUMCONTACTS, 1>
ContactMeasurement<NUMCONTACTS>::get_contact() {
  return contacts_;
}

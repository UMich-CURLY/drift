/* ----------------------------------------------------------------------------
 * Copyright 2022, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

template<unsigned int CONTACT_DIM>
ContactMeasurement<CONTACT_DIM>::ContactMeasurement() : Measurement(CONTACT) {
  contacts_.setZero();
}

template<unsigned int CONTACT_DIM>
void ContactMeasurement<CONTACT_DIM>::set_contact(
    const Eigen::Matrix<bool, CONTACT_DIM, 1>& contacts) {
  contacts_ = contacts;
}

template<unsigned int CONTACT_DIM>
Eigen::Matrix<bool, CONTACT_DIM, 1>
ContactMeasurement<CONTACT_DIM>::get_contact() const {
  return contacts_;
}

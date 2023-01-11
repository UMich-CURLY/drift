/* ----------------------------------------------------------------------------
 * Copyright 2022, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */


ContactMeasurement::ContactMeasurement() : Measurement(CONTACT) {}


void ContactMeasurement::set_contact(
    const Eigen::Matrix<bool, Eigen::Dynamic, 1>& contacts) {
  contacts_ = contacts;
}


Eigen::Matrix<bool, Eigen::Dynamic, 1> ContactMeasurement::get_contact() const {
  return contacts_;
}

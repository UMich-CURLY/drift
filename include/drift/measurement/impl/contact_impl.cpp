/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   contact_impl.cpp
 *  @author Justin Yu
 *  @brief  Implementation for contact measurement
 *  @date   May 16, 2023
 **/

namespace measurement {
ContactMeasurement::ContactMeasurement() : Measurement(CONTACT) {}


void ContactMeasurement::set_contact(
    const Eigen::Matrix<bool, Eigen::Dynamic, 1>& contacts) {
  contacts_ = contacts;
}


Eigen::Matrix<bool, Eigen::Dynamic, 1> ContactMeasurement::get_contact() const {
  return contacts_;
}
}    // namespace measurement

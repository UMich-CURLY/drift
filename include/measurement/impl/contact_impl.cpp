/* ----------------------------------------------------------------------------
 * Copyright 2022, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   contact_impl.cpp
 *  @author Justin Yu
 *  @brief  Implementation for contact measurement
 *  @date   Nov 16, 2022
 **/

ContactMeasurement::ContactMeasurement() : Measurement(CONTACT) {}


void ContactMeasurement::set_contact(
    const Eigen::Matrix<bool, Eigen::Dynamic, 1>& contacts) {
  contacts_ = contacts;
}


Eigen::Matrix<bool, Eigen::Dynamic, 1> ContactMeasurement::get_contact() const {
  return contacts_;
}

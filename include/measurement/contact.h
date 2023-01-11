/* ----------------------------------------------------------------------------
 * Copyright 2022, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   contact.h
 *  @author Justin Yu
 *  @brief  Header file for robot ground contact state
 *  @date   Nov 16, 2022
 **/

#ifndef CONTACT_H
#define CONTACT_H

#include "measurement.h"

/**
 * @class ContactMeasurement
 *
 * Derived measurement class containing robot-ground
 * contact state.
 */

class ContactMeasurement : public Measurement {
 public:
  /**
   * @brief Default constructor.
   */
  ContactMeasurement();

  /**
   * @brief Set the contact state vector for this measurement.
   *
   * @param[in] contacts: vector of booleans containing contact state.
   */
  void set_contact(const Eigen::Matrix<bool, Eigen::Dynamic, 1>& contacts);

  /**
   * @brief Get the contact state vector for this measurement.
   *
   * @return vector of booleans containing contact state.
   */
  Eigen::Matrix<bool, Eigen::Dynamic, 1> get_contact() const;

 private:
  Eigen::Matrix<bool, Eigen::Dynamic, 1> contacts_;
};
#include "measurement/impl/contact_impl.cpp"

#endif    // CONTACT_H

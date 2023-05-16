/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   contact.h
 *  @author Justin Yu
 *  @brief  Header file for robot ground contact state
 *  @date   May 16, 2023
 **/

#ifndef MEASUREMENT_CONTACT_H
#define MEASUREMENT_CONTACT_H

#include "measurement.h"

namespace measurement {
/**
 * @class ContactMeasurement
 *
 * @brief measurement class containing robot-ground
 * contact state.
 */

class ContactMeasurement : public Measurement {
 public:
  /// @name Constructors
  /// @{
  /**
   * @brief Default constructor.
   */
  ContactMeasurement();
  /// @}

  /// @name Setter
  /// @{
  /**
   * @brief Set the contact state vector for this measurement.
   *
   * @param[in] contacts: vector of booleans containing contact state.
   */
  void set_contact(const Eigen::Matrix<bool, Eigen::Dynamic, 1>& contacts);
  /// @}

  /// @name Getter
  /// @{
  /**
   * @brief Get the contact state vector for this measurement.
   *
   * @return vector of booleans containing contact state.
   */
  Eigen::Matrix<bool, Eigen::Dynamic, 1> get_contact() const;
  /// @}

 private:
  Eigen::Matrix<bool, Eigen::Dynamic, 1>
      contacts_;    // boolean 1D matrix representing the status of contact. 1 =
                    // contact, 0 = no contact
};
}    // namespace measurement
#include "drift/measurement/impl/contact_impl.cpp"

#endif    // MEASUREMENT_CONTACT_H

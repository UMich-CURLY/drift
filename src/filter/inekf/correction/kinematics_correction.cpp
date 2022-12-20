#include "filter/inekf/correction/kinematics_correction.h"

namespace inekf {
using namespace std;
using namespace lie_group;

using ContactID = int;

KinematicsCorrection::KinematicsCorrection(
    KinematicsQueuePtr sensor_data_buffer_ptr, const ErrorType& error_type,
    const std::string& aug_type)
    : Correction::Correction(),
      sensor_data_buffer_ptr_(sensor_data_buffer_ptr),
      sensor_data_buffer_(*sensor_data_buffer_ptr.get()),
      error_type_(error_type),
      aug_type_(aug_type) {
  correction_type_ = CorrectionType::KINEMATICS;
}

// Correct state using kinematics measured between body frame and contact point
void KinematicsCorrection::Correct(RobotState& state) {
  Eigen::VectorXd Z, Y, b;
  Eigen::MatrixXd H, N, PI;

  // Initialize containers to store contacts that will be removed or augmented
  // after correction
  vector<ContactID> remove_contacts;
  vectorKinematics new_contacts;
  vector<int> used_contact_ids;

  //---------------------------------------------------------------
  /// TODO: add get_contact in the KinematicsMeasurement() class
  /// TODO: Be sure to also change and check the whole function
  /// TOASK: Should we use map<int, bool> or just Matrix<int, CONTACT_DIM,1> for
  /// contact measurements?
  // --------------------------------------------------------------
  // std::map<int, bool> contacts
  //     = sensor_data_buffer_.front().get()->get_contacts();
  std::map<int, bool> contacts;
  // const vectorKinematics measured_kinematics
  //     = sensor_data_buffer_.front().get()->get_kinematics();
  // sensor_data_buffer_.pop();
  vectorKinematics measured_kinematics;

  for (vectorKinematicsIterator it = measured_kinematics.begin();
       it != measured_kinematics.end(); ++it) {
    // Detect and skip if an ID is not unique (this would cause singularity
    // issues in InEKF::Correct)
    if (find(used_contact_ids.begin(), used_contact_ids.end(), it->id)
        != used_contact_ids.end()) {
      cout << "Duplicate contact ID detected! Skipping measurement.\n";
      continue;
    } else {
      used_contact_ids.push_back(it->id);
    }

    // Find contact indicator for the kinematics measurement
    // The contact data comes from sensor_data_buffer_, which is updated in the
    // main loop
    map<int, bool>::iterator it_contact = contacts.find(it->id);
    if (it_contact == contacts.end()) {
      continue;
    }    // Skip if contact state is unknown
    bool contact_indicated = it_contact->second;

    // See if we can find id for augmented states (in this case, contact id) in
    // the state
    // map<int, int>::iterator it_estimated = aug_id_to_column_id_.find(it->id);
    // bool found = it_estimated != aug_id_to_column_id_.end();

    bool found = aug_id_to_column_id_.count(it->id);

    if (!contact_indicated && found) {
      // If contact is not indicated and id is found in estimated_contacts, then
      // remove state
      remove_contacts.push_back(it->id);    // Add id to remove list
    } else if (contact_indicated && !found) {
      //  If contact is indicated and id is not found i n estimated_contacts,
      //  then augment state
      new_contacts.push_back(*it);    // Add to augment list
    } else if (contact_indicated && found) {
      // If contact is indicated and id is found in estimated_contacts, then
      // correct using kinematics
      int dimX = state.dimX();
      int dimTheta = state.dimTheta();
      int dimP = state.dimP();
      int startIndex;

      // Fill out H
      startIndex = H.rows();
      H.conservativeResize(startIndex + 3, dimP);
      H.block(startIndex, 0, 3, dimP) = Eigen::MatrixXd::Zero(3, dimP);
      if (state.get_state_type() == StateType::WorldCentric) {
        H.block(startIndex, 6, 3, 3) = -Eigen::Matrix3d::Identity();    // -I
        H.block(startIndex, 3 * aug_id_to_column_id_[it->id] - dimTheta, 3, 3)
            = Eigen::Matrix3d::Identity();    // I
      } else {
        H.block(startIndex, 6, 3, 3) = Eigen::Matrix3d::Identity();    // I
        H.block(startIndex, 3 * aug_id_to_column_id_[it->id] - dimTheta, 3, 3)
            = -Eigen::Matrix3d::Identity();    // -I
      }

      // Fill out N
      startIndex = N.rows();
      N.conservativeResize(startIndex + 3, startIndex + 3);
      N.block(startIndex, 0, 3, startIndex)
          = Eigen::MatrixXd::Zero(3, startIndex);
      N.block(0, startIndex, startIndex, 3)
          = Eigen::MatrixXd::Zero(startIndex, 3);
      N.block(startIndex, startIndex, 3, 3)
          = state.get_world_rotation() * it->covariance.block<3, 3>(3, 3)
            * state.get_world_rotation().transpose();

      // Fill out Z
      startIndex = Z.rows();
      Z.conservativeResize(startIndex + 3, Eigen::NoChange);
      Eigen::Matrix3d R = state.get_rotation();
      Eigen::Vector3d p = state.get_position();
      /// TODO: change this with the new get aug method
      Eigen::Vector3d d = state.get_aug_state(aug_id_to_column_id_[it->id]);
      if (state.get_state_type() == StateType::WorldCentric) {
        Z.segment(startIndex, 3) = R * it->pose.block<3, 1>(0, 3) - (d - p);
      } else {
        Z.segment(startIndex, 3)
            = R.transpose() * (it->pose.block<3, 1>(0, 3) - (p - d));
      }

    } else {
      //  If contact is not indicated and id is found in estimated_contacts_,
      //  then skip
      continue;
    }
  }

  // Correct state using stacked observation
  if (Z.rows() > 0) {
    if (state.get_state_type() == StateType::WorldCentric) {
      CorrectRightInvariant(Z, H, N, state, error_type_);
    } else {
      CorrectLeftInvariant(Z, H, N, state, error_type_);
    }
  }

  // Remove contacts from state
  if (remove_contacts.size() > 0) {
    Eigen::MatrixXd X_rem = state.get_X();
    Eigen::MatrixXd P_rem = state.get_P();
    for (ContactID contact_id : remove_contacts) {
      state.del_aug_state(aug_id_to_column_id_[contact_id]);
      aug_id_to_column_id_.erase(contact_id);
    }
  }

  // Augment state with newly detected contacts
  if (new_contacts.size() > 0) {
    Eigen::MatrixXd X_aug = state.get_X();
    Eigen::MatrixXd P_aug = state.get_P();
    for (vectorKinematicsIterator it = new_contacts.begin();
         it != new_contacts.end(); ++it) {
      // Initialize new contact position mean
      Eigen::Vector3d aug_state;
      if (state.get_state_type() == StateType::WorldCentric) {
        aug_state = state.get_position()
                    + state.get_rotation() * it->pose.block<3, 1>(0, 3);
      } else {
        aug_state = state.get_position() - it->pose.block<3, 1>(0, 3);
      }

      // Initialize new contact covariance - TODO:speed up
      Eigen::MatrixXd F = Eigen::MatrixXd::Zero(state.dimP() + 3, state.dimP());
      F.block(0, 0, state.dimP() - state.dimTheta(),
              state.dimP() - state.dimTheta())
          = Eigen::MatrixXd::Identity(
              state.dimP() - state.dimTheta(),
              state.dimP() - state.dimTheta());    // for old X
      F.block(state.dimP() - state.dimTheta() + 3,
              state.dimP() - state.dimTheta(), state.dimTheta(),
              state.dimTheta())
          = Eigen::MatrixXd::Identity(state.dimTheta(),
                                      state.dimTheta());    // for theta
      Eigen::MatrixXd G = Eigen::MatrixXd::Zero(F.rows(), 3);
      // Blocks for new contact
      if ((state.get_state_type() == StateType::WorldCentric
           && error_type_ == ErrorType::RightInvariant)
          || (state.get_state_type() == StateType::BodyCentric
              && error_type_ == ErrorType::LeftInvariant)) {
        F.block(state.dimP() - state.dimTheta(), 6, 3, 3)
            = Eigen::Matrix3d::Identity();
        G.block(G.rows() - state.dimTheta() - 3, 0, 3, 3)
            = state.get_world_rotation();
      } else {
        F.block(state.dimP() - state.dimTheta(), 6, 3, 3)
            = Eigen::Matrix3d::Identity();
        F.block(state.dimP() - state.dimTheta(), 0, 3, 3)
            = skew(-it->pose.block<3, 1>(0, 3));
        G.block(G.rows() - state.dimTheta() - 3, 0, 3, 3)
            = Eigen::Matrix3d::Identity();
      }
      P_aug = (F * P_aug * F.transpose()
               + G * it->covariance.block<3, 3>(3, 3) * G.transpose())
                  .eval();

      // Send the new contact aug state and aug covariance to robot state
      int aug_idx = state.add_aug_state(
          aug_type_, aug_state,
          P_aug.block(state.dimP() - 3, state.dimP() - 3, 3, 3));

      // Add the aug state matrix index to the augment state information
      // mapping
      aug_id_to_column_id_[it->id] = aug_idx;
    }
  }
}
}    // namespace inekf
#include "filter/inekf/correction/legged_kinematics_correction.h"

namespace inekf {
using namespace std;
using namespace lie_group;

LeggedKinematicsCorrection::LeggedKinematicsCorrection(
    LeggedKinematicsQueuePtr sensor_data_buffer_ptr,
    std::shared_ptr<std::mutex> sensor_data_buffer_mutex_ptr,
    const ErrorType& error_type, const std::string& yaml_filepath)
    : Correction::Correction(sensor_data_buffer_mutex_ptr),
      sensor_data_buffer_ptr_(sensor_data_buffer_ptr),
      error_type_(error_type) {
  correction_type_ = CorrectionType::LEGGED_KINEMATICS;
  cout << "Loading legged kinematics correction config from " << yaml_filepath
       << endl;
  YAML::Node config_ = YAML::LoadFile(yaml_filepath);
  encoder_cov_val_ = config_["noises"]["encoder_std"]
                         ? config_["noises"]["encoder_std"].as<double>()
                         : 0.01;
  kinematics_additive_cov_val_
      = config_["noises"]["kinematics_additive_std"]
            ? config_["noises"]["kinematics_additive_std"].as<double>()
            : 0.05;

  double contact_noise_std
      = config_["noises"]["contact_noise_std"]
            ? config_["noises"]["contact_noise_std"].as<double>()
            : 0.1;
  contact_noise_cov_ = contact_noise_std * contact_noise_std
                       * Eigen::Matrix<double, 3, 3>::Identity();
}

const LeggedKinematicsQueuePtr
LeggedKinematicsCorrection::get_sensor_data_buffer_ptr() const {
  return sensor_data_buffer_ptr_;
}

// Correct state using kinematics measured between body frame and contact point
bool LeggedKinematicsCorrection::Correct(RobotState& state) {
  Eigen::VectorXd Z, Y, b;
  Eigen::MatrixXd H, N, PI;

  // Initialize containers to store contacts that will be removed or augmented
  // after correction
  vector<int> remove_contacts;         // leg id of contacts to be removed
  vector<ContactInfo> new_contacts;    // new contacts to be added

  //---------------------------------------------------------------
  /// TODO: add get_contact in the LeggedKinematicsMeasurement() class
  /// TODO: Be sure to also change and check the whole function
  /// TOASK: Should we use map<int, bool> or just Matrix<int, CONTACT_DIM,1> for
  /// contact measurements?
  // --------------------------------------------------------------

  sensor_data_buffer_mutex_ptr_.get()->lock();
  KinematicsMeasurementPtr kinematics_measurement
      = sensor_data_buffer_ptr_->front();
  sensor_data_buffer_ptr_->pop();
  sensor_data_buffer_mutex_ptr_.get()->unlock();

  int num_legs = kinematics_measurement->get_num_legs();
  kinematics_measurement->ComputeKinematics();

  for (int id = 0; id < num_legs; id++) {
    bool has_contact = kinematics_measurement->get_contact(id);
    Eigen::Vector3d pose = kinematics_measurement->get_kin_pos(id);
    Eigen::Matrix3d J = kinematics_measurement->get_J(id);
    Eigen::Matrix3d cov
        = J * encoder_cov_val_ * J.transpose()
          + kinematics_additive_cov_val_ * Eigen::Matrix3d::Identity();

    // bool found = aug_id_to_column_id_.count(id);
    bool found = aug_id_to_column_id_ptr_.count(id);

    if (!has_contact && found) {
      // If contact is not indicated and leg id is found in previous existing
      // contact map, then remove state
      remove_contacts.push_back(id);    // Add id to remove list
    } else if (has_contact && !found) {
      //  If contact is indicated and leg id is not found in previous exisiting
      //  contact map, then augment state
      ContactInfo new_contact;
      new_contact.id = id;
      new_contact.pose = pose;
      new_contact.cov = cov;
      new_contacts.push_back(new_contact);    // Add id to add map
    } else if (has_contact && found) {
      // If contact is indicated and leg id is found in previous existing
      // contact map, then correct using kinematics
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
        H.block(startIndex, 3 * *(aug_id_to_column_id_ptr_[id]) - dimTheta, 3,
                3)
            = Eigen::Matrix3d::Identity();    // I
      } else {
        H.block(startIndex, 6, 3, 3) = Eigen::Matrix3d::Identity();    // I
        H.block(startIndex, 3 * *(aug_id_to_column_id_ptr_[id]) - dimTheta, 3,
                3)
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
          = state.get_world_rotation() * cov
            * state.get_world_rotation().transpose();

      // Fill out Z
      startIndex = Z.rows();
      Z.conservativeResize(startIndex + 3, Eigen::NoChange);
      Eigen::Matrix3d R = state.get_rotation();
      Eigen::Vector3d p = state.get_position();
      /// TODO: change this with the new get aug method
      Eigen::Vector3d d = state.get_aug_state(*(aug_id_to_column_id_ptr_[id]));
      if (state.get_state_type() == StateType::WorldCentric) {
        Z.segment(startIndex, 3) = R * pose - (d - p);
      } else {
        Z.segment(startIndex, 3) = R.transpose() * (pose - (p - d));
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
    for (int id : remove_contacts) {
      state.del_aug_state(*(aug_id_to_column_id_ptr_[id]));
      aug_id_to_column_id_ptr_.erase(id);
    }
  }

  // Augment state with newly detected contacts
  if (new_contacts.size() > 0) {
    Eigen::MatrixXd X_aug = state.get_X();
    Eigen::MatrixXd P_aug = state.get_P();
    for (auto& new_contact : new_contacts) {
      // Initialize new contact position mean
      Eigen::Vector3d aug_state;
      if (state.get_state_type() == StateType::WorldCentric) {
        aug_state
            = state.get_position() + state.get_rotation() * new_contact.pose;
      } else {
        aug_state = state.get_position() - new_contact.pose;
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
            = skew(-new_contact.pose);
        G.block(G.rows() - state.dimTheta() - 3, 0, 3, 3)
            = Eigen::Matrix3d::Identity();
      }
      P_aug = (F * P_aug * F.transpose() + G * new_contact.cov * G.transpose())
                  .eval();

      // Send the new contact aug state and aug covariance to robot state
      aug_id_to_column_id_ptr_[new_contact.id]
          = std::make_shared<int>(state.dimX());
      int aug_idx = state.add_aug_state(
          aug_state, P_aug.block(state.dimP() - 3, state.dimP() - 3, 3, 3),
          contact_noise_cov_, aug_id_to_column_id_ptr_[new_contact.id]);

      // Add the aug state matrix index to the augment state information
      // mapping
      *aug_id_to_column_id_ptr_[new_contact.id] = aug_idx;
    }
  }

  return true;
}
}    // namespace inekf
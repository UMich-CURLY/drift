namespace inekf {
using namespace std;
using namespace lie_group;

using ContactIDandIdx = pair<int, int>;

template<typename sensor_data_t>
KinematicsCorrection<sensor_data_t>::KinematicsCorrection(
    std::shared_ptr<std::queue<sensor_data_t>> sensor_data_buffer,
    const ErrorType& error_type, int aug_map_idx)
    : Correction::Correction(),
      sensor_data_buffer_(sensor_data_buffer),
      error_type_(error_type),
      aug_map_idx_(aug_map_idx) {}

// Correct state using kinematics measured between body frame and contact point
template<typename sensor_data_t>
void KinematicsCorrection<sensor_data_t>::Correct(RobotState& state) {
  Eigen::VectorXd Z, Y, b;
  Eigen::MatrixXd H, N, PI;

  // Initialize containers to store contacts that will be removed or augmented
  // after correction
  vector<ContactIDandIdx> remove_contacts;
  vectorKinematics new_contacts;
  vector<int> used_contact_ids;
  std::map<int, bool> contacts
      = sensor_data_buffer_.get()->front().get_contacts();
  const vectorKinematics measured_kinematics
      = sensor_data_buffer_.get()->front().get_kinematics();
  std::map<int, int> estimated_contact_positions
      = state.get_augmented_map(aug_map_idx_);

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

    // See if we can find id estimated_contact_positions in the state
    map<int, int>::iterator it_estimated
        = estimated_contact_positions.find(it->id);
    bool found = it_estimated != estimated_contact_positions.end();

    if (!contact_indicated && found) {
      // If contact is not indicated and id is found in estimated_contacts, then
      // remove state
      remove_contacts.push_back(*it_estimated);    // Add id to remove list
      state.del_aug_state(*it_estimated);
    } else if (contact_indicated && !found) {
      //  If contact is indicated and id is not found i n estimated_contacts,
      //  then augment state
      new_contacts.push_back(*it);    // Add to augment list
      state.add_aug_state(*it);
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
      if (state.getStateType() == StateType::WorldCentric) {
        H.block(startIndex, 6, 3, 3) = -Eigen::Matrix3d::Identity();    // -I
        H.block(startIndex, 3 * it_estimated->second - dimTheta, 3, 3)
            = Eigen::Matrix3d::Identity();    // I
      } else {
        H.block(startIndex, 6, 3, 3) = Eigen::Matrix3d::Identity();    // I
        H.block(startIndex, 3 * it_estimated->second - dimTheta, 3, 3)
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
          = state.getWorldRotation() * it->covariance.block<3, 3>(3, 3)
            * state.getWorldRotation().transpose();

      // Fill out Z
      startIndex = Z.rows();
      Z.conservativeResize(startIndex + 3, Eigen::NoChange);
      Eigen::Matrix3d R = state.getRotation();
      Eigen::Vector3d p = state.getPosition();
      Eigen::Vector3d d = state.getVector(it_estimated->second);
      if (state.getStateType() == StateType::WorldCentric) {
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
    if (state.getStateType() == StateType::WorldCentric) {
      CorrectRightInvariant(Z, H, N, state, error_type_);
    } else {
      CorrectLeftInvariant(Z, H, N, state, error_type_);
    }
  }

  // Remove contacts from state
  if (remove_contacts.size() > 0) {
    Eigen::MatrixXd X_rem = state.getX();
    Eigen::MatrixXd P_rem = state.getP();
    for (vector<ContactIDandIdx>::iterator it = remove_contacts.begin();
         it != remove_contacts.end(); ++it) {
      state.del_aug_state(*it);
    }
  }


  // Augment state with newly detected contacts
  if (new_contacts.size() > 0) {
    Eigen::MatrixXd X_aug = state.getX();
    Eigen::MatrixXd P_aug = state.getP();
    for (vectorKinematicsIterator it = new_contacts.begin();
         it != new_contacts.end(); ++it) {
      state.add_aug_state(*it);
    }
  }
}
}    // namespace inekf
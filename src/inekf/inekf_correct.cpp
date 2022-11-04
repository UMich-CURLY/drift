#include "inekf/inekf_correct.h"

namespace inekf {
using namespace std;
using namespace lie_group;


// Default constructor
Correction::Correction() : InEKF::InEKF() {}

// Constructor with error type
Correction::Correction(ErrorType error_type) : InEKF::InEKF(error_type) {}

void Correction::Correct(RobotState& state) {
    // Just a skeleton for now
}

// Correct state using kinematics measured between imu and contact point
void Correction::Correct(const vectorKinematics& measured_kinematics, RobotState& state) {
    Eigen::VectorXd Z, Y, b;
    Eigen::MatrixXd H, N, PI;

    vector<pair<int,int> > remove_contacts;
    vectorKinematics new_contacts;
    vector<int> used_contact_ids;

    for (vectorKinematicsIterator it=measured_kinematics.begin(); it!=measured_kinematics.end(); ++it) {
        // Detect and skip if an ID is not unique (this would cause singularity issues in InEKF::Correct)
        if (find(used_contact_ids.begin(), used_contact_ids.end(), it->id) != used_contact_ids.end()) { 
            cout << "Duplicate contact ID detected! Skipping measurement.\n";
            continue; 
        } else { used_contact_ids.push_back(it->id); }

        // Find contact indicator for the kinematics measurement
        map<int,bool>::iterator it_contact = contacts_.find(it->id);
        if (it_contact == contacts_.end()) { continue; } // Skip if contact state is unknown
        bool contact_indicated = it_contact->second;

        // See if we can find id estimated_contact_positions
        map<int,int>::iterator it_estimated = estimated_contact_positions_.find(it->id);
        bool found = it_estimated!=estimated_contact_positions_.end();

        // If contact is not indicated and id is found in estimated_contacts_, then remove state
        if (!contact_indicated && found) {
            remove_contacts.push_back(*it_estimated); // Add id to remove list
        //  If contact is indicated and id is not found i n estimated_contacts_, then augment state
        } else if (contact_indicated && !found) {
            new_contacts.push_back(*it); // Add to augment list

        // If contact is indicated and id is found in estimated_contacts_, then correct using kinematics
        } else if (contact_indicated && found) {
            int dimX = state.dimX();
            int dimTheta = state.dimTheta();
            int dimP = state.dimP();
            int startIndex;

            // Fill out H
            startIndex = H.rows();
            H.conservativeResize(startIndex+3, dimP);
            H.block(startIndex,0,3,dimP) = Eigen::MatrixXd::Zero(3,dimP);
            if (state.getStateType() == StateType::WorldCentric) {
                H.block(startIndex,6,3,3) = -Eigen::Matrix3d::Identity(); // -I
                H.block(startIndex,3*it_estimated->second-dimTheta,3,3) = Eigen::Matrix3d::Identity(); // I
            } else {
                H.block(startIndex,6,3,3) = Eigen::Matrix3d::Identity(); // I
                H.block(startIndex,3*it_estimated->second-dimTheta,3,3) = -Eigen::Matrix3d::Identity(); // -I
            }
            
            // Fill out N
            startIndex = N.rows();
            N.conservativeResize(startIndex+3, startIndex+3);
            N.block(startIndex,0,3,startIndex) = Eigen::MatrixXd::Zero(3,startIndex);
            N.block(0,startIndex,startIndex,3) = Eigen::MatrixXd::Zero(startIndex,3);
            N.block(startIndex,startIndex,3,3) = state.getWorldRotation() * it->covariance.block<3,3>(3,3) * state.getWorldRotation().transpose();
    
            // Fill out Z
            startIndex = Z.rows();
            Z.conservativeResize(startIndex+3, Eigen::NoChange);
            Eigen::Matrix3d R = state.getRotation();
            Eigen::Vector3d p = state.getPosition();
            Eigen::Vector3d d = state.getVector(it_estimated->second);  
            if (state.getStateType() == StateType::WorldCentric) {
                Z.segment(startIndex,3) = R*it->pose.block<3,1>(0,3) - (d - p); 
            } else {
                Z.segment(startIndex,3) = R.transpose()*(it->pose.block<3,1>(0,3) - (p - d)); 
            }

        //  If contact is not indicated and id is found in estimated_contacts_, then skip
        } else {
            continue;
        }
    }

    // Correct state using stacked observation
    if (Z.rows()>0) {
        if (state.getStateType() == StateType::WorldCentric) {
            this->CorrectRightInvariant(Z,H,N,state);
            // this->CorrectRightInvariant(obs);
        } else {
            // this->CorrectLeftInvariant(obs);
            this->CorrectLeftInvariant(Z,H,N,state);
        }
    }

    // Remove contacts from state
    if (remove_contacts.size() > 0) {
        Eigen::MatrixXd X_rem = state.getX(); 
        Eigen::MatrixXd P_rem = state.getP();
        for (vector<pair<int,int> >::iterator it=remove_contacts.begin(); it!=remove_contacts.end(); ++it) {
            // Remove row and column from X
            removeRowAndColumn(X_rem, it->second);
            // Remove 3 rows and columns from P
            int startIndex = 3 + 3*(it->second-3);
            removeRowAndColumn(P_rem, startIndex); // TODO: Make more efficient
            removeRowAndColumn(P_rem, startIndex); // TODO: Make more efficient
            removeRowAndColumn(P_rem, startIndex); // TODO: Make more efficient
            // Update all indices for estimated_landmarks and estimated_contact_positions
            for (map<int,int>::iterator it2=estimated_landmarks_.begin(); it2!=estimated_landmarks_.end(); ++it2) {
                if (it2->second > it->second) 
                    it2->second -= 1;
            }
            for (map<int,int>::iterator it2=estimated_contact_positions_.begin(); it2!=estimated_contact_positions_.end(); ++it2) {
                if (it2->second > it->second) 
                    it2->second -= 1;
            }
            // We also need to update the indices of remove_contacts in the case where multiple contacts are being removed at once
            for (vector<pair<int,int> >::iterator it2=it; it2!=remove_contacts.end(); ++it2) {
                if (it2->second > it->second) 
                    it2->second -= 1;
            }
            // Remove from list of estimated contact positions 
            estimated_contact_positions_.erase(it->first);
        }
        // Update state and covariance
        state.setX(X_rem);
        state.setP(P_rem);
    }


    // Augment state with newly detected contacts
    if (new_contacts.size() > 0) {
        Eigen::MatrixXd X_aug = state.getX(); 
        Eigen::MatrixXd P_aug = state.getP();
        for (vectorKinematicsIterator it=new_contacts.begin(); it!=new_contacts.end(); ++it) {
            // Initialize new landmark mean
            int startIndex = X_aug.rows();
            X_aug.conservativeResize(startIndex+1, startIndex+1);
            X_aug.block(startIndex,0,1,startIndex) = Eigen::MatrixXd::Zero(1,startIndex);
            X_aug.block(0,startIndex,startIndex,1) = Eigen::MatrixXd::Zero(startIndex,1);
            X_aug(startIndex, startIndex) = 1;
            if (state.getStateType() == StateType::WorldCentric) {
                X_aug.block(0,startIndex,3,1) = state.getPosition() + state.getRotation()*it->pose.block<3,1>(0,3);
            } else {
                X_aug.block(0,startIndex,3,1) = state.getPosition() - it->pose.block<3,1>(0,3);
            }

            // Initialize new landmark covariance - TODO:speed up
            Eigen::MatrixXd F = Eigen::MatrixXd::Zero(state.dimP()+3,state.dimP()); 
            F.block(0,0,state.dimP()-state.dimTheta(),state.dimP()-state.dimTheta()) = Eigen::MatrixXd::Identity(state.dimP()-state.dimTheta(),state.dimP()-state.dimTheta()); // for old X
            F.block(state.dimP()-state.dimTheta()+3,state.dimP()-state.dimTheta(),state.dimTheta(),state.dimTheta()) = Eigen::MatrixXd::Identity(state.dimTheta(),state.dimTheta()); // for theta
            Eigen::MatrixXd G = Eigen::MatrixXd::Zero(F.rows(),3);
            // Blocks for new contact
            if ((state.getStateType() == StateType::WorldCentric && error_type_ == ErrorType::RightInvariant) || 
                (state.getStateType() == StateType::BodyCentric && error_type_ == ErrorType::LeftInvariant)) {
                F.block(state.dimP()-state.dimTheta(),6,3,3) = Eigen::Matrix3d::Identity(); 
                G.block(G.rows()-state.dimTheta()-3,0,3,3) = state.getWorldRotation();
            } else {
                F.block(state.dimP()-state.dimTheta(),6,3,3) = Eigen::Matrix3d::Identity(); 
                F.block(state.dimP()-state.dimTheta(),0,3,3) = skew(-it->pose.block<3,1>(0,3)); 
                G.block(G.rows()-state.dimTheta()-3,0,3,3) = Eigen::Matrix3d::Identity();
            }
            P_aug = (F*P_aug*F.transpose() + G*it->covariance.block<3,3>(3,3)*G.transpose()).eval(); 

            // Update state and covariance
            state.setX(X_aug); // TODO: move outside of loop (need to make loop independent of state)
            state.setP(P_aug);

            // Add to list of estimated contact positions
            estimated_contact_positions_.insert(pair<int,int> (it->id, startIndex));
        }
    }
}


// Correct using measured body velocity with the estimated velocity
void Correction::Correct(const Eigen::Vector3d& measured_velocity, const Eigen::Matrix3d& covariance, RobotState &state) {
    Eigen::VectorXd Z, Y, b;
    Eigen::MatrixXd H, N, PI;

    // Fill out observation data
    int dimX = state_.dimX();
    int dimTheta = state_.dimTheta();
    int dimP = state_.dimP();

    // Fill out Y
    // Y.conservativeResize(dimX, Eigen::NoChange);
    // Y.segment(0,dimX) = Eigen::VectorXd::Zero(dimX);
    // Y.segment<3>(0) = measured_velocity;
    // Y(3) = -1;       

    // // Fill out b
    // b.conservativeResize(dimX, Eigen::NoChange);
    // b.segment(0,dimX) = Eigen::VectorXd::Zero(dimX);
    // b(3) = -1;       

    // Fill out H
    H.conservativeResize(3, dimP);
    H.block(0,0,3,dimP) = Eigen::MatrixXd::Zero(3,dimP);
    H.block(0,3,3,3) = Eigen::Matrix3d::Identity(); 

    // Fill out N
    N.conservativeResize(3, 3);
    N = covariance;

    // Fill out PI      
    // PI.conservativeResize(3, dimX);
    // PI.block(0,0,3,dimX) = Eigen::MatrixXd::Zero(3,dimX);
    // PI.block(0,0,3,3) = Eigen::Matrix3d::Identity();

    // Fill out Z
    // Z = X*Y-b = PI*X*Y 
    Eigen::Matrix3d R = state_.getRotation();
    Eigen::Vector3d v = state_.getVelocity();


    int startIndex = Z.rows();
    Z.conservativeResize(startIndex+3, Eigen::NoChange);
    Z.segment(0,3) = R*measured_velocity - v; 


    // Correct state using stacked observation
    if (Z.rows()>0) {
        this->CorrectRightInvariant(Z,H,N,state);
    }
}
}
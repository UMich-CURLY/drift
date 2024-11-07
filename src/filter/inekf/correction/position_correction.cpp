#include "drift/filter/inekf/correction/position_correction.h"
#include <fstream>
using namespace std;
using namespace math::lie_group;

namespace filter::inekf {

PositionCorrection::PositionCorrection(
    OdomQueuePtr sensor_data_buffer_ptr,
    std::shared_ptr<std::mutex> sensor_data_buffer_mutex_ptr,
    const ErrorType& error_type, const std::string& yaml_filepath)
    : Correction::Correction(sensor_data_buffer_mutex_ptr),
      sensor_data_buffer_ptr_(sensor_data_buffer_ptr),
      error_type_(error_type) {
  correction_type_ = CorrectionType::POSITION;

  // Load configuration settings from YAML file
  cout << "Loading position correction config from " << yaml_filepath << endl;
  YAML::Node config_ = YAML::LoadFile(yaml_filepath);

  // Set noise covariance
  double std = config_["noises"]["position_std"]
                   ? config_["noises"]["position_std"].as<double>()
                   : 0.1;

  covariance_
      = std * std
        * Eigen::Matrix<double, 3, 3>::Identity();    // 3D position noise

  // Set thresholds
  t_diff_thres_
      = config_["settings"]["correction_time_threshold"]
            ? config_["settings"]["correction_time_threshold"].as<double>()
            : 0.3;

  std::string est_pose_file
      = "/home/neofelis/drift_gps/log/vanilla_est_pose_log.txt";
  est_pose_outfile_.open(est_pose_file);
  est_pose_outfile_.precision(dbl::max_digits10);
}

PositionCorrection::~PositionCorrection() { est_pose_outfile_.close(); }

const OdomQueuePtr PositionCorrection::get_sensor_data_buffer_ptr() const {
  return sensor_data_buffer_ptr_;
}

bool PositionCorrection::Correct(RobotState& state) {
  Eigen::VectorXd Z;
  Eigen::MatrixXd H, N;

  // Lock the sensor data buffer and check for new measurements
  sensor_data_buffer_mutex_ptr_->lock();
  if (sensor_data_buffer_ptr_->empty()) {
    sensor_data_buffer_mutex_ptr_->unlock();
    return false;
  }

  OdomMeasurementPtr measured_position = sensor_data_buffer_ptr_->front();
  double t_diff = measured_position->get_time() - state.get_propagate_time();

  // Skip measurements that are in the future
  if (t_diff >= 0) {
    sensor_data_buffer_mutex_ptr_->unlock();
    return false;
  }

  // Pop until we get to the most recent measurement within time threshold
  sensor_data_buffer_ptr_->pop();
  sensor_data_buffer_mutex_ptr_->unlock();

  if (t_diff < -t_diff_thres_) {
    while (t_diff < -t_diff_thres_) {
      sensor_data_buffer_mutex_ptr_->lock();
      if (sensor_data_buffer_ptr_->empty()) {
        sensor_data_buffer_mutex_ptr_->unlock();
        return false;
      }
      measured_position = sensor_data_buffer_ptr_->front();
      sensor_data_buffer_ptr_->pop();
      sensor_data_buffer_mutex_ptr_->unlock();

      t_diff = measured_position->get_time() - state.get_propagate_time();
    }
  }

  // Open the file in append mode and write the data
  std::ofstream file("debugging_data.txt", std::ios::app);
  if (file.is_open()) {
    // Ensure fixed decimal format with three decimal places for positions
    file << std::fixed << std::setprecision(6);

    // Get time in seconds with milliseconds precision
    double filter_time = state.get_propagate_time();
    double measurement_time = measured_position->get_time();

    // Write Filter time
    file << "Filter time: " << std::fixed << std::setprecision(6) << filter_time
         << " ";

    // Write Filter state by extracting components explicitly
    Eigen::Vector3d filter_state = state.get_position();
    file << "Filter state: " << std::fixed << std::setprecision(6)
         << filter_state.x() << " " << filter_state.y() << " "
         << filter_state.z() << "; ";

    // Write Measurement time
    file << "Measurement time: " << std::fixed << std::setprecision(6)
         << measurement_time << " ";

    // Write Measurement state by extracting components explicitly
    Eigen::Vector3d measurement_state
        = measured_position->get_transformation().block<3, 1>(0, 3);
    file << "Measurement state: " << std::fixed << std::setprecision(6)
         << measurement_state.x() << " " << measurement_state.y() << " "
         << measurement_state.z() << "\n";

    file.close();
  } else {
    std::cerr << "Unable to open file for writing imu data.\n";
  }
  // Set state time to the measurement time
  state.set_time(measured_position->get_time());

  int dimP = state.dimP();

  // Fill out H
  H.conservativeResize(3, dimP);
  H.block(0, 0, 3, dimP)
      = Eigen::MatrixXd::Zero(3, dimP);                 // Zero out all elements
  H.block(0, 6, 3, 3) = Eigen::Matrix3d::Identity();    // Place identity in the
                                                        // 7th, 8th, 9th columns

  N.conservativeResize(3, 3);
  N = state.get_rotation().transpose() * covariance_ * state.get_rotation();


  Eigen::Vector3d position_error
      = measured_position->get_transformation().block<3, 1>(0, 3)
        - state.get_position();

  // Log the propagated position
  Eigen::Vector3d propagate_position = state.get_position();

  Eigen::Vector3d measured_posit
      = measured_position->get_transformation().block<3, 1>(0, 3);

  // Fill Z with stacked position error
  Eigen::Matrix3d R = state.get_rotation();
  int startIndex = Z.rows();
  Z.conservativeResize(startIndex + 3, Eigen::NoChange);

  Z.segment(0, 3) = R.transpose()
                    * (measured_position->get_transformation().block<3, 1>(0, 3)
                       - state.get_position());

  // Correct state using Left Invariant EKF
  if (Z.rows() > 0) {
    CorrectLeftInvariant(Z, H, N, state, error_type_);
  }

  // Log the estimated position
  Eigen::Vector3d est_position = state.get_position();

  est_pose_outfile_ << "Propagated: " << propagate_position(0) << ","
                    << propagate_position(1) << "," << propagate_position(2)
                    << " "
                    << "Measured: " << measured_posit(0) << ","
                    << measured_posit(1) << "," << measured_posit(2) << " "
                    << "Estimated: " << est_position(0) << ","
                    << est_position(1) << "," << est_position(2) << std::endl
                    << std::flush;

  return true;
}

bool PositionCorrection::initialize(RobotState& state) {
  // Initialization with first available position data
  while (sensor_data_buffer_ptr_->empty()) {
    return false;
  }

  sensor_data_buffer_mutex_ptr_->lock();
  while (sensor_data_buffer_ptr_->size() > 1) {
    sensor_data_buffer_ptr_->pop();    // Discard old data
  }
  OdomMeasurementPtr measured_position = sensor_data_buffer_ptr_->front();
  sensor_data_buffer_ptr_->pop();
  sensor_data_buffer_mutex_ptr_->unlock();

  // Set the state with the measured position
  state.set_position(measured_position->get_transformation().block<3, 1>(0, 3));
  state.set_rotation(measured_position->get_transformation().block<3, 3>(0, 0));
  state.set_time(measured_position->get_time());

  return true;
}

void PositionCorrection::clear() {
  sensor_data_buffer_mutex_ptr_->lock();
  while (!sensor_data_buffer_ptr_->empty()) {
    sensor_data_buffer_ptr_->pop();
  }
  sensor_data_buffer_mutex_ptr_->unlock();
}

}    // namespace filter::inekf

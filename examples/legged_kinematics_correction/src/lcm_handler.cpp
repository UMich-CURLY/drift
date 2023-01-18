#include "communication/lcm_handler.hpp"


namespace cheetah_inekf_lcm {
lcm_handler::lcm_handler(lcm::LCM* lcm, cheetah_lcm_data_t* cheetah_buffer,
                         boost::mutex* cdata_mtx, bool* reinit_cmd)
    : lcm_(lcm),
      cheetah_buffer_(cheetah_buffer),
      cdata_mtx_(cdata_mtx),
      reinit_cmd_(reinit_cmd) {
  assert(lcm_);    // confirm a nullptr wasn't passed in

  char resolved_path[PATH_MAX];
  char* p = realpath("../", resolved_path);
  config_setting = YAML::LoadFile(
      std::string(resolved_path)
      + "examples/legged_kinematics_correction/config/settings.yaml");
  config_inekf = YAML::LoadFile(
      std::string(resolved_path)
      + "examples/legged_kinematics_correction/config/inekf.yaml");

  /// NOSYNC:
  std::string mode = config_setting["settings"]["mode"]
                         ? config_setting["settings"]["mode"].as<std::string>()
                         : "normal";
  std::string lcm_leg_channel
      = config_setting["settings"]["lcm_leg_channel"]
            ? config_setting["settings"]["lcm_leg_channel"].as<std::string>()
            : "leg_control_data";
  std::string lcm_imu_channel
      = config_setting["settings"]["lcm_imu_channel"]
            ? config_setting["settings"]["lcm_imu_channel"].as<std::string>()
            : "microstrain";
  std::string lcm_contact_channel
      = config_setting["settings"]["lcm_contact_est_channel"]
            ? config_setting["settings"]["lcm_contact_est_channel"]
                  .as<std::string>()
            : "wbc_lcm_data";
  std::string lcm_synced_channel
      = config_setting["settings"]["lcm_synced_channel"]
            ? config_setting["settings"]["lcm_synced_channel"].as<std::string>()
            : "synced_proprioceptive_data";

  bool run_synced = config_setting["settings"]["run_synced"]
                        ? config_setting["settings"]["run_synced"].as<bool>()
                        : false;

  std::string lcm_reinitialize_channel
      = config_setting["settings"]["lcm_reinitialize_channel"]
            ? config_setting["settings"]["lcm_reinitialize_channel"]
                  .as<std::string>()
            : "reinitialize_command";
  lcm_->subscribe(lcm_reinitialize_channel,
                  &cheetah_inekf_lcm::lcm_handler::receiveReinitializeMsg,
                  this);

  if (!run_synced && mode == "normal") {
    lcm_->subscribe(lcm_leg_channel,
                    &cheetah_inekf_lcm::lcm_handler::receiveLegControlMsg,
                    this);
    lcm_->subscribe(lcm_imu_channel,
                    &cheetah_inekf_lcm::lcm_handler::receiveMicrostrainMsg,
                    this);
    lcm_->subscribe(lcm_contact_channel,
                    &cheetah_inekf_lcm::lcm_handler::receiveContactMsg, this);
  }

  /// SYNCED:
  if (run_synced)
    lcm_->subscribe(lcm_synced_channel,
                    &cheetah_inekf_lcm::lcm_handler::synced_msgs_lcm_callback,
                    this);


  start_time_ = 0;

  // data dimensions:
  q_dim = config_setting["settings"]["leg_q_dimension"]
              ? config_setting["settings"]["leg_q_dimension"].as<int>()
              : 12;
  qd_dim = config_setting["settings"]["leg_qd_dimension"]
               ? config_setting["settings"]["leg_qd_dimension"].as<int>()
               : 12;
  tau_est_dim = config_setting["settings"]["leg_tau_dimension"]
                    ? config_setting["settings"]["leg_tau_dimension"].as<int>()
                    : 12;
  acc_dim = config_setting["settings"]["imu_acc_dimension"]
                ? config_setting["settings"]["imu_acc_dimension"].as<int>()
                : 3;
  omega_dim = config_setting["settings"]["imu_omega_dimension"]
                  ? config_setting["settings"]["imu_omega_dimension"].as<int>()
                  : 3;
  rpy_dim = config_setting["settings"]["imu_rpy_dimension"]
                ? config_setting["settings"]["imu_rpy_dimension"].as<int>()
                : 3;
  quat_dim = config_setting["settings"]["imu_quat_dimension"]
                 ? config_setting["settings"]["imu_quat_dimension"].as<int>()
                 : 4;

  seq_imu_data_ = 0;
  seq_joint_state_ = 0;
  seq_contact_ = 0;

  // Set private variables
  double encoder_std, kinematic_prior_orientation_std,
      kinematic_prior_position_std;

  encoder_std = config_inekf["inekf"]["encoder_std"]
                    ? config_inekf["inekf"]["encoder_std"].as<double>()
                    : 0.0174533;
  kinematic_prior_orientation_std
      = config_inekf["inekf"]["kinematic_prior_orientation_std"]
            ? config_inekf["inekf"]["kinematic_prior_orientation_std"]
                  .as<double>()
            : 0.174533;
  kinematic_prior_position_std
      = config_inekf["inekf"]["kinematic_prior_position_std"]
            ? config_inekf["inekf"]["kinematic_prior_position_std"].as<double>()
            : 0.05;

  cov_encoders_
      = encoder_std * encoder_std * Eigen::Matrix<double, 12, 12>::Identity();
  cov_prior_ = Eigen::Matrix<double, 6, 6>::Identity();
  cov_prior_.block<3, 3>(0, 0) = kinematic_prior_orientation_std
                                 * kinematic_prior_orientation_std
                                 * Eigen::Matrix<double, 3, 3>::Identity();
  cov_prior_.block<3, 3>(3, 3) = kinematic_prior_position_std
                                 * kinematic_prior_position_std
                                 * Eigen::Matrix<double, 3, 3>::Identity();
}

lcm_handler::~lcm_handler() {}

void lcm_handler::receiveReinitializeMsg(const lcm::ReceiveBuffer* rbuf,
                                         const std::string& chan,
                                         const reinitialization_lcmt* msg) {
  *reinit_cmd_ = msg->reinitialize_cmd;
}


void lcm_handler::receiveLegControlMsg(const lcm::ReceiveBuffer* rbuf,
                                       const std::string& chan,
                                       const leg_control_data_lcmt* msg) {
  // std::cout << "Received leg data" << std::endl;
  if (start_time_ == 0) {
    start_time_ = rbuf->recv_utime;
  }
  if (cheetah_buffer_->contact_q.size()
      > cheetah_buffer_->joint_state_q.size()) {
    std::shared_ptr<JointStateMeasurement> joint_state_ptr
        = std::shared_ptr<JointStateMeasurement>(
            new JointStateMeasurement(q_dim));

    /// LEG:
    joint_state_ptr.get()->joint_position
        = Eigen::Map<const Eigen::MatrixXf>(msg->q, q_dim, 1).cast<double>();
    joint_state_ptr.get()->joint_velocity
        = Eigen::Map<const Eigen::MatrixXf>(msg->qd, qd_dim, 1).cast<double>();
    joint_state_ptr.get()->joint_effort
        = Eigen::Map<const Eigen::MatrixXf>(msg->tau_est, tau_est_dim, 1)
              .cast<double>();

    /// LOW: 500Hz version:
    boost::mutex::scoped_lock lock(*cdata_mtx_);
    cheetah_buffer_->joint_state_q.push(joint_state_ptr);
  }
}

void lcm_handler::receiveMicrostrainMsg(const lcm::ReceiveBuffer* rbuf,
                                        const std::string& chan,
                                        const microstrain_lcmt* msg) {
  /// LOW: 500Hz version:
  // std::cout << "Received IMU data" << std::endl;
  if (start_time_ == 0) {
    start_time_ = rbuf->recv_utime;
  }
  if (cheetah_buffer_->joint_state_q.size() > cheetah_buffer_->imu_q.size()) {
    // std::shared_ptr<LcmIMUStruct> microstrain_data =
    // std::make_shared<LcmIMUStruct>();
    std::shared_ptr<ImuMeasurement<double>> imu_measurement_ptr
        = std::shared_ptr<ImuMeasurement<double>>(new ImuMeasurement<double>);

    imu_measurement_ptr.get()->orientation.w = msg->quat[0];
    imu_measurement_ptr.get()->orientation.x = msg->quat[1];
    imu_measurement_ptr.get()->orientation.y = msg->quat[2];
    imu_measurement_ptr.get()->orientation.z = msg->quat[3];
    imu_measurement_ptr.get()->angular_velocity.x = msg->omega[0];
    imu_measurement_ptr.get()->angular_velocity.y = msg->omega[1];
    imu_measurement_ptr.get()->angular_velocity.z = msg->omega[2];
    imu_measurement_ptr.get()->linear_acceleration.x = msg->acc[0];
    imu_measurement_ptr.get()->linear_acceleration.y = msg->acc[1];
    imu_measurement_ptr.get()->linear_acceleration.z = msg->acc[2];

    double timestamp = (1.0 * (rbuf->recv_utime - start_time_)) / pow(10, 6);

    boost::mutex::scoped_lock lock(*cdata_mtx_);
    cheetah_buffer_->timestamp_q.push(timestamp);
    cheetah_buffer_->imu_q.push(imu_measurement_ptr);
  }
}

void lcm_handler::receiveContactMsg(const lcm::ReceiveBuffer* rbuf,
                                    const std::string& chan,
                                    const wbc_test_data_t* msg) {
  // std::cout << "Received contact data" << std::endl;
  std::shared_ptr<ContactsMeasurement> contact_ptr
      = std::shared_ptr<ContactsMeasurement>(new ContactsMeasurement);

  /// CONTACTS:
  Eigen::Matrix<bool, 4, 1> contacts;
  for (int i = 0; i < 4; ++i) {
    // std::cout << msg->contact_est[i];
    contacts[i] = msg->contact_est[i];
  }
  // std::cout << std::endl;
  contact_ptr->setContacts(contacts);
  boost::mutex::scoped_lock lock(*cdata_mtx_);
  cheetah_buffer_->contact_q.push(contact_ptr);
}

// synchronized version (works if and only if you already have a set of
// synchronized messages):
void lcm_handler::synced_msgs_lcm_callback(
    const lcm::ReceiveBuffer* rbuf, const std::string& channel_name,
    const synced_proprioceptive_lcmt* msg) {
  // ROS_DEBUG_STREAM("Receive new synchronized msg");
  // std::cout << "Receive new synchronized msg" << std::endl;
  seq_joint_state_++;
  std::shared_ptr<ImuMeasurement<double>> imu_measurement_ptr
      = std::shared_ptr<ImuMeasurement<double>>(new ImuMeasurement<double>);
  std::shared_ptr<JointStateMeasurement> joint_state_ptr
      = std::shared_ptr<JointStateMeasurement>(
          new JointStateMeasurement(q_dim));
  std::shared_ptr<ContactsMeasurement> contact_ptr
      = std::shared_ptr<ContactsMeasurement>(new ContactsMeasurement);

  /// TIMESTAMP:
  double timestamp = msg->timestamp;

  /// IMU:
  imu_measurement_ptr.get()->orientation.w = msg->quat[0];
  imu_measurement_ptr.get()->orientation.x = msg->quat[1];
  imu_measurement_ptr.get()->orientation.y = msg->quat[2];
  imu_measurement_ptr.get()->orientation.z = msg->quat[3];
  imu_measurement_ptr.get()->angular_velocity.x = msg->omega[0];
  imu_measurement_ptr.get()->angular_velocity.y = msg->omega[1];
  imu_measurement_ptr.get()->angular_velocity.z = msg->omega[2];
  imu_measurement_ptr.get()->linear_acceleration.x = msg->acc[0];
  imu_measurement_ptr.get()->linear_acceleration.y = msg->acc[1];
  imu_measurement_ptr.get()->linear_acceleration.z = msg->acc[2];

  /// LEG:
  joint_state_ptr.get()->joint_position
      = Eigen::Map<const Eigen::MatrixXf>(msg->q, q_dim, 1).cast<double>();
  joint_state_ptr.get()->joint_velocity
      = Eigen::Map<const Eigen::MatrixXf>(msg->qd, qd_dim, 1).cast<double>();
  joint_state_ptr.get()->joint_effort
      = Eigen::Map<const Eigen::MatrixXf>(msg->tau_est, tau_est_dim, 1)
            .cast<double>();

  /// CONTACTS:
  Eigen::Matrix<bool, 4, 1> contacts;
  for (int i = 0; i < msg->num_legs; ++i) {
    // std::cout << msg->contact[i];
    contacts[i] = msg->contact[i];
  }
  // std::cout << std::endl;
  // std::cout << "Corresponding contacts: " << contacts[0] << contacts[1] <<
  // contacts[2] << contacts[3] << std::endl;

  contact_ptr->setContacts(contacts);

  // push into queues:
  boost::mutex::scoped_lock lock(*cdata_mtx_);
  cheetah_buffer_->timestamp_q.push(timestamp);
  cheetah_buffer_->imu_q.push(imu_measurement_ptr);
  cheetah_buffer_->joint_state_q.push(joint_state_ptr);
  cheetah_buffer_->contact_q.push(contact_ptr);
}

}    // namespace cheetah_inekf_lcm

// template class cheetah_inekf_lcm::KinematicsHandler;
// template class cheetah_inekf_lcm::lcm_handler;

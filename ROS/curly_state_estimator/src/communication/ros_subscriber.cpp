/* ----------------------------------------------------------------------------
 * Copyright 2022, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   ros_subsriber.cpp
 *  @author Tzu-Yuan Lin
 *  @brief  Source file for ROS subscriber class
 *  @date   December 6, 2022
 **/

#include "communication/ros_subscriber.h"

namespace ros_wrapper {


ROSSubscriber::ROSSubscriber(ros::NodeHandle* nh)
    : nh_(nh), thread_started_(false) {}


ROSSubscriber::~ROSSubscriber() {
  if (thread_started_ == true) {
    subscribing_thread_.join();
  }
  subscriber_list_.clear();
  contact_subscriber_list_.clear();
  joint_state_subscriber_list_.clear();
  imu_queue_list_.clear();
}


IMUQueuePair ROSSubscriber::AddIMUSubscriber(const std::string topic_name) {
  // Create a new queue for data buffers
  IMUQueuePtr imu_queue_ptr(new IMUQueue);

  // Initialize a new mutex for this subscriber
  mutex_list_.emplace_back(new std::mutex);

  // Create the subscriber
  subscriber_list_.push_back(nh_->subscribe<sensor_msgs::Imu>(
      topic_name, 1000,
      boost::bind(&ROSSubscriber::IMUCallback, this, _1, mutex_list_.back(),
                  imu_queue_ptr)));

  // Keep the ownership of the data queue in this class
  imu_queue_list_.push_back(imu_queue_ptr);

  return {imu_queue_ptr, mutex_list_.back()};
}

GPSVelQueuePair ROSSubscriber::AddGPSVelocitySubscriber(
    const std::string topic_name) {
  // Create a new queue for data buffers
  GPSVelQueuePtr gps_vel_queue_ptr(new GPSQueue);

  // Initialize a new mutex for this subscriber
  mutex_list_.emplace_back(new std::mutex);

  // Create the subscriber
  subscriber_list_.push_back(nh_->subscribe<sensor_msgs::Imu>(
      topic_name, 1000,
      boost::bind(&ROSSubscriber::GPSVelCallback, this, _1, mutex_list_.back(),
                  gps_vel_queue_ptr)));

  // Keep the ownership of the data queue in this class
  gps_vel_queue_list_.push_back(gps_vel_queue_ptr);

  return {gps_vel_queue_ptr, mutex_list_.back()};
}

LegKinQueuePair ROSSubscriber::AddMiniCheetahKinematicsSubscriber(
    const std::string contact_topic_name,
    const std::string encoder_topic_name) {
  // Create a new queue for data buffers
  LegKinQueuePtr kin_queue_ptr(new LegKinQueue);

  // Initialize a new mutex for this subscriber
  mutex_list_.emplace_back(new std::mutex);

  contact_subscriber_list_.push_back(
      std::make_shared<ContactMsgFilterT>(*nh_, contact_topic_name, 1));
  joint_state_subscriber_list_.push_back(
      std::make_shared<JointStateMsgFilterT>(*nh_, encoder_topic_name, 1));


  // ApproximateTime takes a queue size as its constructor argument, hence
  // LegKinSyncPolicy(10)
  leg_kin_sync_list_.push_back(
      std::make_shared<message_filters::Synchronizer<LegKinSyncPolicy>>(
          LegKinSyncPolicy(10), *contact_subscriber_list_.back(),
          *joint_state_subscriber_list_.back()));

  leg_kin_sync_list_.back()->registerCallback(
      boost::bind(&ROSSubscriber::MiniCheetahKinCallBack, this, _1, _2,
                  mutex_list_.back(), kin_queue_ptr));

  // Keep the ownership of the data queue in this class
  kin_queue_list_.push_back(kin_queue_ptr);

  return {kin_queue_ptr, mutex_list_.back()};
}

VelocityQueuePair ROSSubscriber::AddDifferentialDriveVelocitySubscriber(
    const std::string topic_name) {
  // Create a new queue for data buffers
  VelocityQueuePtr vel_queue_ptr(new VelocityQueue);

  // Initialize a new mutex for this subscriber
  mutex_list_.emplace_back(new std::mutex);

  // Create the subscriber
  subscriber_list_.push_back(nh_->subscribe<sensor_msgs::JointState>(
      topic_name, 1000,
      boost::bind(&ROSSubscriber::DifferentialEncoder2VelocityCallback, this,
                  _1, mutex_list_.back(), vel_queue_ptr)));

  // Keep the ownership of the data queue in this class
  vel_queue_list_.push_back(vel_queue_ptr);

  return {vel_queue_ptr, mutex_list_.back()};
};


void ROSSubscriber::StartSubscribingThread() {
  subscribing_thread_ = std::thread([this] { this->RosSpin(); });
  thread_started_ = true;
}

void ROSSubscriber::IMUCallback(
    const boost::shared_ptr<const sensor_msgs::Imu>& imu_msg,
    const std::shared_ptr<std::mutex>& mutex, IMUQueuePtr& imu_queue) {
  // Create an imu measurement object
  std::shared_ptr<ImuMeasurement<double>> imu_measurement(
      new ImuMeasurement<double>);

  // Set headers and time stamps
  imu_measurement->set_header(
      imu_msg->header.seq,
      imu_msg->header.stamp.sec + imu_msg->header.stamp.nsec / 1000000000.0,
      imu_msg->header.frame_id);
  // Set angular velocity
  imu_measurement->set_ang_vel(imu_msg->angular_velocity.x,
                               imu_msg->angular_velocity.y,
                               imu_msg->angular_velocity.z);
  // Set linear acceleration
  imu_measurement->set_lin_acc(imu_msg->linear_acceleration.x,
                               imu_msg->linear_acceleration.y,
                               imu_msg->linear_acceleration.z);
  // Set orientation estimate
  imu_measurement->set_quaternion(
      imu_msg->orientation.w, imu_msg->orientation.x, imu_msg->orientation.y,
      imu_msg->orientation.z);

  // std::lock_guard<std::mutex> lock(*mutex);
  mutex.get()->lock();
  imu_queue->push(imu_measurement);
  mutex.get()->unlock();
  // std::cout << "mutex id: " << mutex.get() << std::endl;
}

void ROSSubscriber::GPSVelCallback(
    const boost::shared_ptr<const geometry_msgs::TwistStamped>& gps_vel_msg,
    const std::shared_ptr<std::mutex>& mutex, VelocityQueuePtr& gps_vel_queue) {
  // Create an imu measurement object
  std::shared_ptr<VelocityMeasurement<double>> gps_vel_measurement(
      new VelocityMeasurement<double>);

  // Set headers and time stamps
  gps_vel_measurement->set_header(
      gps_vel_msg->header.seq,
      gps_vel_msg->header.stamp.sec
          + gps_vel_msg->header.stamp.nsec / 1000000000.0,
      gps_vel_msg->header.frame_id);

  // Set linear velocity
  gps_vel_measurement->set_velocity(gps_vel_msg->linear_velocity.x,
                                    gps_vel_msg->linear_velocity.y,
                                    gps_vel_msg->linear_velocity.z);

  // Set angular velocity
  gps_vel_measurement->set_ang_velocity(gps_vel_msg->angular_velocity.x,
                                        gps_vel_msg->angular_velocity.y,
                                        gps_vel_msg->angular_velocity.z);

  // std::lock_guard<std::mutex> lock(*mutex);
  mutex.get()->lock();
  gps_vel_queue->push(gps_vel_measurement);
  mutex.get()->unlock();
  // std::cout << "mutex id: " << mutex.get() << std::endl;
}

void ROSSubscriber::DifferentialEncoder2VelocityCallback(
    const boost::shared_ptr<const sensor_msgs::JointState>& encoder_msg,
    const std::shared_ptr<std::mutex>& mutex, VelocityQueuePtr& vel_queue) {
  // Create an velocity measurement object
  std::shared_ptr<VelocityMeasurement<double>> vel_measurement(
      new VelocityMeasurement<double>);

  // Set headers and time stamps
  vel_measurement->set_header(
      encoder_msg->header.seq,
      encoder_msg->header.stamp.sec
          + encoder_msg->header.stamp.nsec / 1000000000.0,
      encoder_msg->header.frame_id);

  double wheel_radius = 0.1651;

  double vr = (encoder_msg->velocity[1] + encoder_msg->velocity[3]) / 2.0
              * wheel_radius;
  double vl = (encoder_msg->velocity[0] + encoder_msg->velocity[2]) / 2.0
              * wheel_radius;
  double vx = (vr + vl) / 2.0;

  vel_measurement->set_velocity(vx, 0, 0);
  vel_queue->push(vel_measurement);
}

void ROSSubscriber::MiniCheetahKinCallBack(
    const boost::shared_ptr<const custom_sensor_msgs::ContactArray>&
        contact_msg,
    const boost::shared_ptr<const sensor_msgs::JointState>& encoder_msg,
    const std::shared_ptr<std::mutex>& mutex, LegKinQueuePtr& kin_queue) {
  // Create a legged kinematics measurement object
  // #include "communication/kinematics_impl.cpp"
  // Set headers and time stamps
  // TODO: Figure out how headers are set for a kinematics measurement
  std::shared_ptr<MiniCheetahKinematics> kin_measurement(
      new MiniCheetahKinematics);
  /// TODO: Idealy, use the timestamp used by Approximate time synchronizer,
  //  or use the one with lower frequency
  kin_measurement->set_header(
      contact_msg->header.seq,
      contact_msg->header.stamp.sec
          + contact_msg->header.stamp.nsec / 1000000000.0,
      contact_msg->header.frame_id);

  Eigen::Matrix<bool, 4, 1> ctmsg;
  ctmsg << contact_msg->contacts[0].indicator,
      contact_msg->contacts[1].indicator, contact_msg->contacts[2].indicator,
      contact_msg->contacts[3].indicator;
  kin_measurement->set_contact(ctmsg);

  Eigen::Matrix<double, 12, 1> jsmsg;
  jsmsg << encoder_msg->position[0], encoder_msg->position[1],
      encoder_msg->position[2], encoder_msg->position[3],
      encoder_msg->position[4], encoder_msg->position[5],
      encoder_msg->position[6], encoder_msg->position[7],
      encoder_msg->position[8], encoder_msg->position[9],
      encoder_msg->position[10], encoder_msg->position[11];
  kin_measurement->set_joint_state(jsmsg);

  kin_queue->push(kin_measurement);
}

void ROSSubscriber::RosSpin() {
  while (ros::ok()) {
    ros::spinOnce();
  }
  // spinner_.spin();
}

}    // namespace ros_wrapper

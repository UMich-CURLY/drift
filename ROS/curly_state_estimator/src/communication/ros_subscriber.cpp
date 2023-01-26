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
  mfilter_subscriber_list_.clear();
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

LegKinQueuePair ROSSubscriber::AddKinematicsSubscriber(
    const std::string contact_topic_name,
    const std::string encoder_topic_name) {
  // Create a new queue for data buffers
  LegKinQueuePtr kin_queue_ptr(new LegKinQueue);

  // Initialize a new mutex for this subscriber
  mutex_list_.emplace_back(new std::mutex);

  // Create the subscriber
  // subscriber_list_.push_back(nh_->subscribe<sensor_msgs::JointState>(
  //     encoder_topic_name, 1000,
  //     boost::bind(&ROSSubscriber::kin_call_back, this, _1,
  //     mutex_list_.back(),
  //                 kin_queue_ptr)));

  message_filters::Subscriber<custom_sensor_msgs::ContactArray> contact_sub(
      *nh_, contact_topic_name, 1);
  message_filters::Subscriber<sensor_msgs::JointState> encoder_sub(
      *nh_, encoder_topic_name, 1);

  mfilter_subscriber_list_.push_back(contact_sub);
  mfilter_subscriber_list_.push_back(encoder_sub);

  typedef message_filters::sync_policies::ApproximateTime<
      custom_sensor_msgs::ContactArray, sensor_msgs::JointState>
      MySyncPolicy;

  // ApproximateTime takes a queue size as its constructor argument, hence
  // MySyncPolicy(10)
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),
                                                   contact_sub, encoder_sub);
  sync.registerCallback(boost::bind(&ROSSubscriber::KinCallBack, this, _1, _2,
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

void ROSSubscriber::KinCallBack(
    const boost::shared_ptr<const custom_sensor_msgs::ContactArray>&
        contact_msg,
    const boost::shared_ptr<const sensor_msgs::JointState>& encoder_msg,
    const std::shared_ptr<std::mutex>& mutex, LegKinQueuePtr& kin_queue) {
  // Create a legged kinematics measurement object
  // #include "communication/kinematics_impl.cpp"
  // Set headers and time stamps
  // TODO: Figure out how headers are set for a kinematics measurement
  std::shared_ptr<LeggedKinematics> kin_measurement(new LeggedKinematics);
  kin_measurement->set_header(
      contact_msg->header.seq,
      contact_msg->header.stamp.sec
          + contact_msg->header.stamp.nsec / 1000000000.0,
      contact_msg->header.frame_id);

  Eigen::Matrix<bool, Eigen::Dynamic, 1> ctmsg;
  ctmsg << contact_msg->contacts[0].indicator,
      contact_msg->contacts[1].indicator, contact_msg->contacts[2].indicator,
      contact_msg->contacts[3].indicator;
  kin_measurement->set_contact(ctmsg);
  Eigen::Matrix<double, Eigen::Dynamic, 1> jsmsg;
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

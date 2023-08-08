/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   ros_subsriber.cpp
 *  @author Tzu-Yuan Lin, Tingjun Li, Justin Yu
 *  @brief  Source file for ROS subscriber class
 *  @date   May 16, 2023
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
  std::cout << "Subscribing to IMU topic: " << topic_name << std::endl;

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

IMUQueuePair ROSSubscriber::AddFetchIMUSubscriber(
    const std::string imu_topic_name, const std::string offset_topic_name) {
  std::cout << "Subscribing to IMU topic: " << imu_topic_name << std::endl;
  // Create a new queue for data buffers
  IMUQueuePtr imu_queue_ptr(new IMUQueue);

  // Initialize a new mutex for this subscriber
  mutex_list_.emplace_back(new std::mutex);

  imu_subscriber_list_.push_back(
      std::make_shared<IMUMsgFilterT>(*nh_, imu_topic_name, 1));
  imu_offset_subscriber_list_.push_back(
      std::make_shared<IMUOffsetMsgFilterT>(*nh_, offset_topic_name, 1));


  // ApproximateTime takes a queue size as its constructor argument, hence
  // IMUSyncPolicy(10)
  imu_sync_list_.push_back(
      std::make_shared<message_filters::Synchronizer<IMUSyncPolicy>>(
          IMUSyncPolicy(10), *imu_subscriber_list_.back(),
          *imu_offset_subscriber_list_.back()));

  imu_sync_list_.back()->registerCallback(
      boost::bind(&ROSSubscriber::FetchIMUCallBack, this, _1, _2,
                  mutex_list_.back(), imu_queue_ptr));

  // Keep the ownership of the data queue in this class
  imu_queue_list_.push_back(imu_queue_ptr);

  return {imu_queue_ptr, mutex_list_.back()};
}

LeggedKinQueuePair ROSSubscriber::AddMiniCheetahKinematicsSubscriber(
    const std::string contact_topic_name,
    const std::string encoder_topic_name) {
  std::cout << "Subscribing to contact topic: " << contact_topic_name
            << std::endl;
  std::cout << "Subscribing to encoder topic: " << encoder_topic_name
            << std::endl;
  // Create a new queue for data buffers
  LeggedKinQueuePtr kin_queue_ptr(new LeggedKinQueue);

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

VelocityQueuePair ROSSubscriber::AddVelocitySubscriber(
    const std::string topic_name) {
  // Create a new queue for data buffers
  VelocityQueuePtr vel_queue_ptr(new VelocityQueue);

  // Initialize a new mutex for this subscriber
  mutex_list_.emplace_back(new std::mutex);

  // Create the subscriber
  subscriber_list_.push_back(nh_->subscribe<geometry_msgs::TwistStamped>(
      topic_name, 1000,
      boost::bind(&ROSSubscriber::VelocityCallback, this, _1,
                  mutex_list_.back(), vel_queue_ptr)));

  // Keep the ownership of the data queue in this class
  vel_queue_list_.push_back(vel_queue_ptr);

  return {vel_queue_ptr, mutex_list_.back()};
}

VelocityQueuePair ROSSubscriber::AddVelocityWithCovarianceSubscriber(
    const std::string topic_name) {
  // Create a new queue for data buffers
  VelocityQueuePtr vel_queue_ptr(new VelocityQueue);

  // Initialize a new mutex for this subscriber
  mutex_list_.emplace_back(new std::mutex);

  // Create the subscriber
  subscriber_list_.push_back(
      nh_->subscribe<geometry_msgs::TwistWithCovarianceStamped>(
          topic_name, 1000,
          boost::bind(&ROSSubscriber::VelocityWithCovarianceCallback, this, _1,
                      mutex_list_.back(), vel_queue_ptr)));

  // Keep the ownership of the data queue in this class
  vel_queue_list_.push_back(vel_queue_ptr);

  return {vel_queue_ptr, mutex_list_.back()};
}

VelocityQueuePair ROSSubscriber::AddDifferentialDriveVelocitySubscriber(
    const std::string topic_name, double wheel_radius) {
  std::cout << "Subscribing to wheel encoder topic: " << topic_name
            << std::endl;
  // Create a new queue for data buffers
  VelocityQueuePtr vel_queue_ptr(new VelocityQueue);

  // Initialize a new mutex for this subscriber
  mutex_list_.emplace_back(new std::mutex);

  // Create the subscriber
  subscriber_list_.push_back(nh_->subscribe<sensor_msgs::JointState>(
      topic_name, 1000,
      boost::bind(&ROSSubscriber::DifferentialEncoder2VelocityCallback, this,
                  _1, mutex_list_.back(), vel_queue_ptr, wheel_radius)));

  // Keep the ownership of the data queue in this class
  vel_queue_list_.push_back(vel_queue_ptr);

  return {vel_queue_ptr, mutex_list_.back()};
};

std::tuple<VelocityQueuePtr, std::shared_ptr<std::mutex>,
           AngularVelocityQueuePtr, std::shared_ptr<std::mutex>>
ROSSubscriber::AddDifferentialDriveVelocitySubscriber(
    const std::string topic_name, double wheel_radius, double track_width) {
  std::cout << "Subscribing to wheel encoder topic: " << topic_name
            << std::endl;
  // Create a new queue for data buffers
  VelocityQueuePtr vel_queue_ptr(new VelocityQueue);
  AngularVelocityQueuePtr ang_vel_queue_ptr(new AngularVelocityQueue);

  // Initialize a new mutex for this subscriber
  mutex_list_.emplace_back(new std::mutex);
  auto vel_mutex = mutex_list_.back();
  mutex_list_.emplace_back(new std::mutex);
  auto ang_vel_mutex = mutex_list_.back();

  // Create the subscriber
  subscriber_list_.push_back(nh_->subscribe<sensor_msgs::JointState>(
      topic_name, 1000,
      boost::bind(&ROSSubscriber::DifferentialEncoder2VelocityCallback, this,
                  _1, vel_mutex, ang_vel_mutex, vel_queue_ptr,
                  ang_vel_queue_ptr, wheel_radius, track_width)));

  // Keep the ownership of the data queue in this class
  vel_queue_list_.push_back(vel_queue_ptr);
  ang_vel_queue_list_.push_back(ang_vel_queue_ptr);

  return {vel_queue_ptr, vel_mutex, ang_vel_queue_ptr, ang_vel_mutex};
};


VelocityQueuePair
ROSSubscriber::AddDifferentialDriveLinearVelocitySubscriber_Fetch(
    const std::string topic_name, double wheel_raidus) {
  std::cout << "Subscribing to wheel encoder topic: " << topic_name
            << std::endl;
  // Create a new queue for data buffers
  VelocityQueuePtr vel_queue_ptr(new VelocityQueue);

  // Initialize a new mutex for this subscriber
  mutex_list_.emplace_back(new std::mutex);

  // Create the subscriber
  subscriber_list_.push_back(nh_->subscribe<sensor_msgs::JointState>(
      topic_name, 1000,
      boost::bind(
          &ROSSubscriber::DifferentialEncoder2LinearVelocityCallback_Fetch,
          this, _1, mutex_list_.back(), vel_queue_ptr, wheel_raidus)));

  // Keep the ownership of the data queue in this class
  vel_queue_list_.push_back(vel_queue_ptr);

  return {vel_queue_ptr, mutex_list_.back()};
};

VelocityQueuePair ROSSubscriber::AddOdom2VelocityCallback(
    const std::string topic_name,
    const std::vector<double>& translation_odomsrc2body,
    const std::vector<double>& rotation_odomsrc2body) {
  std::cout << "Subscribing to odometry topic: " << topic_name << std::endl;
  // Create a new queue for data buffers
  VelocityQueuePtr vel_queue_ptr(new VelocityQueue);

  // Initialize a new mutex for this subscriber
  mutex_list_.emplace_back(new std::mutex);

  // Calculate the transformation from odometry source to body
  odom_src_to_body_ = Eigen::Matrix4d::Identity();
  Eigen::Quaternion<double> orientation_quat(
      rotation_odomsrc2body[0], rotation_odomsrc2body[1],
      rotation_odomsrc2body[2], rotation_odomsrc2body[3]);
  odom_src_to_body_.block<3, 3>(0, 0) = orientation_quat.toRotationMatrix();
  odom_src_to_body_.block<3, 1>(0, 3) = Eigen::Vector3d(
      {translation_odomsrc2body[0], translation_odomsrc2body[1],
       translation_odomsrc2body[2]});

  // Create a new pair of Odometry queue and its mutex in case we have multiple
  // odometry sources
  //   OdomMeasurementPtr odom_ptr(new OdomMeasurement);
  prev_odom_map_[odom_src_id_] = nullptr;

  // Create the subscriber
  subscriber_list_.push_back(nh_->subscribe<nav_msgs::Odometry>(
      topic_name, 1000,
      boost::bind(&ROSSubscriber::Odom2VelocityCallback, this, _1,
                  mutex_list_.back(), vel_queue_ptr, odom_src_id_)));

  odom_src_id_ += 1;

  // Keep the ownership of the data queue in this class
  vel_queue_list_.push_back(vel_queue_ptr);

  return {vel_queue_ptr, mutex_list_.back()};
};

std::tuple<VelocityQueuePtr, std::shared_ptr<std::mutex>,
           AngularVelocityQueuePtr, std::shared_ptr<std::mutex>>
ROSSubscriber::AddDifferentialDriveVelocitySubscriber_Fetch(
    const std::string topic_name, double wheel_radius, double track_width) {
  std::cout << "Subscribing to wheel encoder topic: " << topic_name
            << std::endl;
  // Create a new queue for data buffers
  VelocityQueuePtr vel_queue_ptr(new VelocityQueue);
  AngularVelocityQueuePtr ang_vel_queue_ptr(new AngularVelocityQueue);

  // Initialize a new mutex for this subscriber
  mutex_list_.emplace_back(new std::mutex);
  auto vel_mutex = mutex_list_.back();
  mutex_list_.emplace_back(new std::mutex);
  auto ang_vel_mutex = mutex_list_.back();

  // Create the subscriber
  subscriber_list_.push_back(nh_->subscribe<sensor_msgs::JointState>(
      topic_name, 1000,
      boost::bind(&ROSSubscriber::DifferentialEncoder2VelocityCallback_Fetch,
                  this, _1, vel_mutex, ang_vel_mutex, vel_queue_ptr,
                  ang_vel_queue_ptr, wheel_radius, track_width)));

  // Keep the ownership of the data queue in this class
  vel_queue_list_.push_back(vel_queue_ptr);
  ang_vel_queue_list_.push_back(ang_vel_queue_ptr);

  return {vel_queue_ptr, vel_mutex, ang_vel_queue_ptr, ang_vel_mutex};
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
  imu_measurement->set_angular_velocity(imu_msg->angular_velocity.x,
                                        imu_msg->angular_velocity.y,
                                        imu_msg->angular_velocity.z);
  // Set linear acceleration
  imu_measurement->set_lin_acc(imu_msg->linear_acceleration.x,
                               imu_msg->linear_acceleration.y,
                               imu_msg->linear_acceleration.z);
  // Set orientation estimate
  // Check if IMU has quaternion data:
  if (Eigen::Vector4d({imu_msg->orientation.w, imu_msg->orientation.x,
                       imu_msg->orientation.y, imu_msg->orientation.z})
          .norm()
      != 0) {
    imu_measurement->set_quaternion(
        imu_msg->orientation.w, imu_msg->orientation.x, imu_msg->orientation.y,
        imu_msg->orientation.z);
  }

  mutex.get()->lock();
  imu_queue->push(imu_measurement);
  mutex.get()->unlock();
}

void ROSSubscriber::VelocityCallback(
    const boost::shared_ptr<const geometry_msgs::TwistStamped>& vel_msg,
    const std::shared_ptr<std::mutex>& mutex, VelocityQueuePtr& vel_queue) {
  // Create an velocity measurement object
  std::shared_ptr<VelocityMeasurement<double>> vel_measurement(
      new VelocityMeasurement<double>);

  // Set headers and time stamps
  vel_measurement->set_header(
      vel_msg->header.seq,
      vel_msg->header.stamp.sec + vel_msg->header.stamp.nsec / 1000000000.0,
      vel_msg->header.frame_id);

  vel_measurement->set_velocity(vel_msg->twist.linear.x,
                                vel_msg->twist.linear.y,
                                vel_msg->twist.linear.z);
  mutex.get()->lock();
  vel_queue->push(vel_measurement);
  mutex.get()->unlock();
}

void ROSSubscriber::VelocityWithCovarianceCallback(
    const boost::shared_ptr<const geometry_msgs::TwistWithCovarianceStamped>&
        vel_msg,
    const std::shared_ptr<std::mutex>& mutex, VelocityQueuePtr& vel_queue) {
  // Create an velocity measurement object
  std::shared_ptr<VelocityMeasurement<double>> vel_measurement(
      new VelocityMeasurement<double>);

  // Set headers and time stamps
  vel_measurement->set_header(
      vel_msg->header.seq,
      vel_msg->header.stamp.sec + vel_msg->header.stamp.nsec / 1000000000.0,
      vel_msg->header.frame_id);

  vel_measurement->set_velocity(vel_msg->twist.twist.linear.x,
                                vel_msg->twist.twist.linear.y,
                                vel_msg->twist.twist.linear.z);
  mutex.get()->lock();
  vel_queue->push(vel_measurement);
  mutex.get()->unlock();
}


void ROSSubscriber::DifferentialEncoder2VelocityCallback(
    const boost::shared_ptr<const sensor_msgs::JointState>& encoder_msg,
    const std::shared_ptr<std::mutex>& mutex, VelocityQueuePtr& vel_queue,
    double wheel_radius) {
  // Create an velocity measurement object
  std::shared_ptr<VelocityMeasurement<double>> vel_measurement(
      new VelocityMeasurement<double>);

  // Set headers and time stamps
  vel_measurement->set_header(
      encoder_msg->header.seq,
      encoder_msg->header.stamp.sec
          + encoder_msg->header.stamp.nsec / 1000000000.0,
      encoder_msg->header.frame_id);

  double vr = (encoder_msg->velocity[1] + encoder_msg->velocity[3]) / 2.0
              * wheel_radius;
  double vl = (encoder_msg->velocity[0] + encoder_msg->velocity[2]) / 2.0
              * wheel_radius;
  double vx = (vr + vl) / 2.0;

  vel_measurement->set_velocity(vx, 0, 0);
  mutex.get()->lock();
  vel_queue->push(vel_measurement);
  mutex.get()->unlock();
}

void ROSSubscriber::DifferentialEncoder2VelocityCallback(
    const boost::shared_ptr<const sensor_msgs::JointState>& encoder_msg,
    const std::shared_ptr<std::mutex>& vel_mutex,
    const std::shared_ptr<std::mutex>& ang_vel_mutex,
    VelocityQueuePtr& vel_queue, AngularVelocityQueuePtr& ang_vel_queue,
    double wheel_radius, double track_width) {
  // Create an velocity measurement object
  std::shared_ptr<VelocityMeasurement<double>> vel_measurement(
      new VelocityMeasurement<double>);
  std::shared_ptr<AngularVelocityMeasurement<double>> ang_vel_measurement(
      new AngularVelocityMeasurement<double>);

  // Set headers and time stamps
  vel_measurement->set_header(
      encoder_msg->header.seq,
      encoder_msg->header.stamp.sec
          + encoder_msg->header.stamp.nsec / 1000000000.0,
      encoder_msg->header.frame_id);

  ang_vel_measurement->set_header(
      encoder_msg->header.seq,
      encoder_msg->header.stamp.sec
          + encoder_msg->header.stamp.nsec / 1000000000.0,
      encoder_msg->header.frame_id);

  double vr = (encoder_msg->velocity[1] + encoder_msg->velocity[3]) / 2.0
              * wheel_radius;
  double vl = (encoder_msg->velocity[0] + encoder_msg->velocity[2]) / 2.0
              * wheel_radius;
  double vx = (vr + vl) / 2.0;
  double omega_z = (vr - vl) / track_width;

  vel_measurement->set_velocity(vx, 0, 0);
  ang_vel_measurement->set_angular_velocity(0, 0, omega_z);

  vel_mutex.get()->lock();
  vel_queue->push(vel_measurement);
  vel_mutex.get()->unlock();

  ang_vel_mutex.get()->lock();
  ang_vel_queue->push(ang_vel_measurement);
  ang_vel_mutex.get()->unlock();
}

void ROSSubscriber::DifferentialEncoder2LinearVelocityCallback_Fetch(
    const boost::shared_ptr<const sensor_msgs::JointState>& encoder_msg,
    const std::shared_ptr<std::mutex>& vel_mutex, VelocityQueuePtr& vel_queue,
    double wheel_radius) {
  // Create an velocity measurement object
  std::shared_ptr<VelocityMeasurement<double>> vel_measurement(
      new VelocityMeasurement<double>);

  // Set headers and time stamps
  vel_measurement->set_header(
      encoder_msg->header.seq,
      encoder_msg->header.stamp.sec
          + encoder_msg->header.stamp.nsec / 1000000000.0,
      encoder_msg->header.frame_id);

  // Fetch
  if (encoder_msg->velocity.size() <= 2) {
    // velocity message from wheel encoder is in an array of size greater than 2
    return;
  }

  double vr = encoder_msg->velocity[1] * wheel_radius;
  double vl = encoder_msg->velocity[0] * wheel_radius;
  double vx = (vr + vl) / 2.0;

  vel_measurement->set_velocity(vx, 0, 0);
  vel_mutex.get()->lock();
  vel_queue->push(vel_measurement);
  vel_mutex.get()->unlock();
}

void ROSSubscriber::DifferentialEncoder2VelocityCallback_Fetch(
    const boost::shared_ptr<const sensor_msgs::JointState>& encoder_msg,
    const std::shared_ptr<std::mutex>& vel_mutex,
    const std::shared_ptr<std::mutex>& ang_vel_mutex,
    VelocityQueuePtr& vel_queue, AngularVelocityQueuePtr& ang_vel_queue,
    double wheel_radius, double track_width) {
  // Create an velocity measurement object
  std::shared_ptr<VelocityMeasurement<double>> vel_measurement(
      new VelocityMeasurement<double>);
  std::shared_ptr<AngularVelocityMeasurement<double>> ang_vel_measurement(
      new AngularVelocityMeasurement<double>);

  // Set headers and time stamps
  vel_measurement->set_header(
      encoder_msg->header.seq,
      encoder_msg->header.stamp.sec
          + encoder_msg->header.stamp.nsec / 1000000000.0,
      encoder_msg->header.frame_id);

  ang_vel_measurement->set_header(
      encoder_msg->header.seq,
      encoder_msg->header.stamp.sec
          + encoder_msg->header.stamp.nsec / 1000000000.0,
      encoder_msg->header.frame_id);

  // Fetch
  if (encoder_msg->velocity.size() <= 2) {
    // velocity message from wheel encoder is in an array of size greater than 2
    return;
  }

  double vr = encoder_msg->velocity[1] * wheel_radius;
  double vl = encoder_msg->velocity[0] * wheel_radius;
  double vx = (vr + vl) / 2.0;
  double omega_z = (vr - vl) / track_width;

  vel_measurement->set_velocity(vx, 0, 0);
  ang_vel_measurement->set_angular_velocity(0, 0, omega_z);
  vel_mutex.get()->lock();
  vel_queue->push(vel_measurement);
  vel_mutex.get()->unlock();

  ang_vel_mutex.get()->lock();
  ang_vel_queue->push(ang_vel_measurement);
  ang_vel_mutex.get()->unlock();
}

void ROSSubscriber::MiniCheetahKinCallBack(
    const boost::shared_ptr<const custom_sensor_msgs::ContactArray>&
        contact_msg,
    const boost::shared_ptr<const sensor_msgs::JointState>& encoder_msg,
    const std::shared_ptr<std::mutex>& mutex, LeggedKinQueuePtr& kin_queue) {
  // Create a legged kinematics measurement object
  // Set headers and time stamps
  std::shared_ptr<kinematics::MiniCheetahKinematics> kin_measurement(
      new kinematics::MiniCheetahKinematics);
  kin_measurement->set_header(
      contact_msg->header.seq,
      contact_msg->header.stamp.sec
          + contact_msg->header.stamp.nsec / 1000000000.0,
      contact_msg->header.frame_id);

  Eigen::Matrix<bool, 4, 1> ct_msg;
  ct_msg << contact_msg->contacts[0].indicator,
      contact_msg->contacts[1].indicator, contact_msg->contacts[2].indicator,
      contact_msg->contacts[3].indicator;
  kin_measurement->set_contact(ct_msg);

  Eigen::Matrix<double, 12, 1> js_msg;
  js_msg << encoder_msg->position[0], encoder_msg->position[1],
      encoder_msg->position[2], encoder_msg->position[3],
      encoder_msg->position[4], encoder_msg->position[5],
      encoder_msg->position[6], encoder_msg->position[7],
      encoder_msg->position[8], encoder_msg->position[9],
      encoder_msg->position[10], encoder_msg->position[11];
  kin_measurement->set_joint_state(js_msg);

  Eigen::Matrix<double, 12, 1> jsvel_msg;
  jsvel_msg << encoder_msg->velocity[0], encoder_msg->velocity[1],
      encoder_msg->velocity[2], encoder_msg->velocity[3],
      encoder_msg->velocity[4], encoder_msg->velocity[5],
      encoder_msg->velocity[6], encoder_msg->velocity[7],
      encoder_msg->velocity[8], encoder_msg->velocity[9],
      encoder_msg->velocity[10], encoder_msg->velocity[11];
  kin_measurement->set_joint_state_velocity(jsvel_msg);

  mutex.get()->lock();
  kin_queue->push(kin_measurement);
  mutex.get()->unlock();
}

void ROSSubscriber::FetchIMUCallBack(
    const boost::shared_ptr<const sensor_msgs::Imu>& imu_msg,
    const boost::shared_ptr<const geometry_msgs::Vector3Stamped>&
        imu_offset_msg,
    const std::shared_ptr<std::mutex>& mutex, IMUQueuePtr& imu_queue) {
  // Set headers and time stamps
  std::shared_ptr<ImuMeasurement<double>> imu_measurement(
      new ImuMeasurement<double>);

  // Set headers and time stamps
  imu_measurement->set_header(
      imu_msg->header.seq,
      imu_msg->header.stamp.sec + imu_msg->header.stamp.nsec / 1000000000.0,
      imu_msg->header.frame_id);
  // Set angular velocity
  imu_measurement->set_angular_velocity(
      imu_msg->angular_velocity.x + imu_offset_msg->vector.x,
      imu_msg->angular_velocity.y + imu_offset_msg->vector.y,
      imu_msg->angular_velocity.z + imu_offset_msg->vector.z);
  // Set linear acceleration
  imu_measurement->set_lin_acc(imu_msg->linear_acceleration.x,
                               imu_msg->linear_acceleration.y,
                               imu_msg->linear_acceleration.z);
  // Set orientation estimate
  // Check if IMU has quaternion data:
  if (Eigen::Vector4d({imu_msg->orientation.w, imu_msg->orientation.x,
                       imu_msg->orientation.y, imu_msg->orientation.z})
          .norm()
      != 0) {
    imu_measurement->set_quaternion(
        imu_msg->orientation.w, imu_msg->orientation.x, imu_msg->orientation.y,
        imu_msg->orientation.z);
  }

  // std::lock_guard<std::mutex> lock(*mutex);
  mutex.get()->lock();
  imu_queue->push(imu_measurement);
  mutex.get()->unlock();
}

void ROSSubscriber::Odom2VelocityCallback(
    const boost::shared_ptr<const nav_msgs::Odometry>& odom_msg,
    const std::shared_ptr<std::mutex>& vel_mutex, VelocityQueuePtr& vel_queue,
    int odom_src_id) {
  Eigen::Vector3d translation(odom_msg->pose.pose.position.x,
                              odom_msg->pose.pose.position.y,
                              odom_msg->pose.pose.position.z);
  Eigen::Quaterniond quat(
      odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
      odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);

  auto odom_ptr = std::make_shared<OdomMeasurement>(
      translation, quat, odom_msg->header.seq, odom_msg->header.stamp.toSec(),
      odom_msg->header.frame_id);

  // We need two odometry data to calculate the velocity
  auto& prev_odom_ptr = prev_odom_map_[odom_src_id];
  if (prev_odom_ptr == nullptr) {
    prev_odom_map_[odom_src_id] = odom_ptr;
    return;
  }

  // When we have at least two odometry data, we can calculate the velocity
  auto prev_transformation = prev_odom_ptr->get_transformation();
  double prev_time = prev_odom_ptr->get_time();
  auto curr_transformation = odom_ptr->get_transformation();
  double curr_time = odom_ptr->get_time();

  double time_diff = curr_time - prev_time;

  // Set current odometry as previous odometry
  prev_odom_map_[odom_src_id] = odom_ptr;

  Eigen::Matrix4d transformation = odom_src_to_body_.inverse()
                                   * prev_transformation.inverse()
                                   * curr_transformation * odom_src_to_body_;
  Eigen::Matrix4d twist_se3 = transformation.log();

  std::shared_ptr<VelocityMeasurement<double>> vel_measurement(
      new VelocityMeasurement<double>);

  // Set headers and time stamps
  vel_measurement->set_header(
      odom_msg->header.seq,
      odom_msg->header.stamp.sec + odom_msg->header.stamp.nsec / 1000000000.0,
      odom_msg->header.frame_id);

  double vx = twist_se3(0, 3) / time_diff;
  double vy = twist_se3(1, 3) / time_diff;
  double vz = twist_se3(2, 3) / time_diff;

  vel_measurement->set_velocity(vx, vy, vz);

  vel_mutex.get()->lock();
  vel_queue->push(vel_measurement);
  vel_mutex.get()->unlock();
}


void ROSSubscriber::RosSpin() {
  //   while (ros::ok()) {
  //     ros::spinOnce();
  //   }
  // spinner_.spin();

  ros::AsyncSpinner spinner(4);    // Use 4 threads
  spinner.start();
  ros::waitForShutdown();
}

}    // namespace ros_wrapper

/* ----------------------------------------------------------------------------
 * Copyright 2022, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   ros_subsriber.h
 *  @author Tzu-Yuan Lin
 *  @brief  Header file for ROS subscriber class
 *  @date   December 6, 2022
 **/

#ifndef ROS_COMMUNICATION_ROS_SUBSCRIBER_H
#define ROS_COMMUNICATION_ROS_SUBSCRIBER_H

#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include "boost/bind.hpp"
#include "custom_sensor_msgs/Contact.h"
#include "custom_sensor_msgs/ContactArray.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"

#include "kinematics/mini_cheetah_kinematics.h"
#include "measurement/contact.h"
#include "measurement/imu.h"
#include "measurement/joint_state.h"
#include "measurement/legged_kinematics.h"
#include "measurement/velocity.h"

typedef std::queue<std::shared_ptr<ImuMeasurement<double>>> IMUQueue;
typedef std::shared_ptr<IMUQueue> IMUQueuePtr;
typedef std::pair<IMUQueuePtr, std::shared_ptr<std::mutex>> IMUQueuePair;

typedef std::queue<std::shared_ptr<VelocityMeasurement<double>>> VelocityQueue;
typedef std::shared_ptr<VelocityQueue> VelocityQueuePtr;
typedef std::pair<VelocityQueuePtr, std::shared_ptr<std::mutex>>
    VelocityQueuePair;

// Legged kinematics sync
typedef std::queue<std::shared_ptr<LeggedKinematicsMeasurement>> LegKinQueue;
typedef std::shared_ptr<LegKinQueue> LegKinQueuePtr;
typedef std::pair<LegKinQueuePtr, std::shared_ptr<std::mutex>> LegKinQueuePair;
typedef message_filters::Subscriber<custom_sensor_msgs::ContactArray>
    ContactMsgFilterT;
typedef message_filters::Subscriber<sensor_msgs::JointState>
    JointStateMsgFilterT;
typedef std::shared_ptr<
    message_filters::Subscriber<custom_sensor_msgs::ContactArray>>
    ContactMsgFilterTPtr;
typedef std::shared_ptr<message_filters::Subscriber<sensor_msgs::JointState>>
    JointStateMsgFilterTPtr;
typedef message_filters::sync_policies::ApproximateTime<
    custom_sensor_msgs::ContactArray, sensor_msgs::JointState>
    LegKinSyncPolicy;
typedef std::shared_ptr<message_filters::Synchronizer<LegKinSyncPolicy>>
    LegKinSyncPtr;

// IMU sync
typedef message_filters::Subscriber<sensor_msgs::Imu> IMUMsgFilterT;
typedef message_filters::Subscriber<geometry_msgs::Vector3Stamped>
    IMUOffsetMsgFilterT;
typedef std::shared_ptr<IMUMsgFilterT> IMUMsgFilterTPtr;
typedef std::shared_ptr<IMUOffsetMsgFilterT> IMUOffsetMsgFilterTPtr;
typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Imu, geometry_msgs::Vector3Stamped>
    IMUSyncPolicy;
typedef std::shared_ptr<message_filters::Synchronizer<IMUSyncPolicy>>
    IMUSyncPtr;

namespace ros_wrapper {
class ROSSubscriber {
 public:
  /// @name Constructors
  /// @{
  /**
   * @brief Construct a new ROSSubscriber object
   *
   * @param[in] nh ROS node handle
   */
  ROSSubscriber(ros::NodeHandle* nh);
  /// @}

  /// @name Destructors
  /// @{
  /**
   * @brief Destroy the ROSSubscriber object
   */
  ~ROSSubscriber();
  /// @}

  /**
   * @brief Add IMU subscriber
   *
   * @param[in] topic_name IMU topic name
   * @return IMUQueuePair IMU queue pair
   */
  IMUQueuePair AddIMUSubscriber(const std::string topic_name);

  /**
   * @brief Add IMU subscriber for Fetch
   *
   * @param[in] imu_topic_name IMU topic name
   * @param[in] offset_topic_name imu offset topic name
   * @return IMUQueuePair IMU queue pair
   */
  IMUQueuePair AddFetchIMUSubscriber(const std::string imu_topic_name,
                                     const std::string offset_topic_name);

  /**
   * @brief Add velocity subscriber
   *
   * @param[in] topic_name velocity topic name
   * @return VelocityQueuePair velocity queue pair
   */
  VelocityQueuePair AddVelocitySubscriber(const std::string topic_name);

  /**
   * @brief Add differential drive velocity subscriber
   *
   * @param[in] topic_name differential drive velocity topic name
   * @return VelocityQueuePair velocity queue pair
   */
  VelocityQueuePair AddDifferentialDriveVelocitySubscriber(
      const std::string topic_name);

  /**
   * @brief Add differential drive velocity subscriber for Husky (4 driving
   * wheels)
   *
   * @param[in] topic_name differential drive velocity topic name
   * @return VelocityQueuePair velocity queue pair
   */
  VelocityQueuePair AddDifferentialDriveVelocitySubscriber_Husky(
      const std::string topic_name);

  /**
   * @brief Add differential drive velocity subscriber (2 driving wheels)
   *
   * @param[in] topic_name differential drive velocity topic name
   * @return VelocityQueuePair velocity queue pair
   */
  VelocityQueuePair AddDifferentialDriveVelocitySubscriber_Fetch(
      const std::string topic_name);

  /**
   * @brief Add differential drive velocity subscriber
   *
   * @param[in] contact_topic_name differential drive velocity topic name
   * @param[in] encoder_topic_name encoder topic name
   * @return VelocityQueuePair velocity queue pair
   */
  LegKinQueuePair AddMiniCheetahKinematicsSubscriber(
      const std::string contact_topic_name,
      const std::string encoder_topic_name);

  /**
   * @brief Start the subscribing thread
   */
  void StartSubscribingThread();

 private:
  /// @name Callback functions
  /// @{
  /**
   * @brief IMU callback function
   *
   * @param[in] imu_msg: IMU message
   * @param[in] mutex: mutex for the buffer queue
   * @param[in] imu_queue: pointer to the buffer queue
   */
  void IMUCallback(const boost::shared_ptr<const sensor_msgs::Imu>& imu_msg,
                   const std::shared_ptr<std::mutex>& mutex,
                   IMUQueuePtr& imu_queue);

  /**
   * @brief Velocity callback function
   *
   * @param[in] vel_msg: velocity message
   * @param[in] mutex: mutex for the buffer queue
   * @param[in] vel_queue: pointer to the buffer queue
   */
  void VelocityCallback(
      const boost::shared_ptr<const geometry_msgs::Twist>& vel_msg,
      const std::shared_ptr<std::mutex>& mutex, VelocityQueuePtr& vel_queue);

  /**
   * @brief Differential encoder to velocity callback function
   *
   * @param encoder_msg: encoder message
   * @param mutex: mutex for the buffer queue
   * @param vel_queue: pointer to the buffer queue
   */
  void DifferentialEncoder2VelocityCallback(
      const boost::shared_ptr<const sensor_msgs::JointState>& encoder_msg,
      const std::shared_ptr<std::mutex>& mutex, VelocityQueuePtr& vel_queue);

  /**
   * @brief Differential encoder to velocity callback function (Husky version, 4
   * driving wheels)
   *
   * @param encoder_msg: encoder message
   * @param mutex: mutex for the buffer queue
   * @param vel_queue: pointer to the buffer queue
   */
  void DifferentialEncoder2VelocityCallback_Husky(
      const boost::shared_ptr<const sensor_msgs::JointState>& encoder_msg,
      const std::shared_ptr<std::mutex>& mutex, VelocityQueuePtr& vel_queue);

  /**
   * @brief Differential encoder to velocity callback function (Fetch version, 2
   * driving wheels)
   *
   * @param encoder_msg: encoder message
   * @param mutex: mutex for the buffer queue
   * @param vel_queue: pointer to the buffer queue
   */
  void DifferentialEncoder2VelocityCallback_Fetch(
      const boost::shared_ptr<const sensor_msgs::JointState>& encoder_msg,
      const std::shared_ptr<std::mutex>& mutex, VelocityQueuePtr& vel_queue);

  /**
   * @brief Mini cheetah kinematics callback function
   *
   * @param contact_msg: Contact message
   * @param encoder_msg: encoder message
   * @param mutex: mutex for the buffer queue
   * @param vel_queue: pointer to the buffer queue
   */
  void MiniCheetahKinCallBack(
      const boost::shared_ptr<const custom_sensor_msgs::ContactArray>&
          contact_msg,
      const boost::shared_ptr<const sensor_msgs::JointState>& encoder_msg,
      const std::shared_ptr<std::mutex>& mutex, LegKinQueuePtr& kin_queue);

  /**
   * @brief fetch imu callback function
   *
   * @param imu_msg: imu message
   * @param imu_offset_msg: imu offset message
   * @param mutex: mutex for the buffer queue
   * @param imu_queue: pointer to the buffer queue
   *
   */
  void FetchIMUCallBack(
      const boost::shared_ptr<const sensor_msgs::Imu>& imu_msg,
      const boost::shared_ptr<const geometry_msgs::Vector3Stamped>&
          imu_offset_msg,
      const std::shared_ptr<std::mutex>& mutex, IMUQueuePtr& imu_queue);

  void RosSpin();

  ros::NodeHandle* nh_;                             // The ROS handle
  std::vector<ros::Subscriber> subscriber_list_;    // List of subscribers

  std::vector<ContactMsgFilterTPtr> contact_subscriber_list_;
  std::vector<JointStateMsgFilterTPtr> joint_state_subscriber_list_;

  std::vector<IMUMsgFilterTPtr> imu_subscriber_list_;
  std::vector<IMUOffsetMsgFilterTPtr> imu_offset_subscriber_list_;

  // measurement queue list
  std::vector<IMUQueuePtr> imu_queue_list_;    // List of IMU queue pointers
  std::vector<VelocityQueuePtr>
      vel_queue_list_;    // List of velocity queue pointers
  std::vector<LegKinQueuePtr>
      kin_queue_list_;    // List of kinematics queue pointers
  std::vector<LegKinSyncPtr> leg_kin_sync_list_;
  std::vector<IMUSyncPtr> imu_sync_list_;
  std::vector<std::shared_ptr<std::mutex>> mutex_list_;    // List of mutexes

  bool thread_started_;    // Flag of the thread started, true for started,
                           // false for not started
  std::thread subscribing_thread_;

  ros::MultiThreadedSpinner spinner_;
};

}    // namespace ros_wrapper

#endif
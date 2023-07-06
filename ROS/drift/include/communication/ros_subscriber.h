/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
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
#include <tuple>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <unsupported/Eigen/MatrixFunctions>
#include "boost/bind.hpp"
#include "custom_sensor_msgs/Contact.h"
#include "custom_sensor_msgs/ContactArray.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"

#include "drift/estimator/inekf_estimator.h"
#include "drift/kinematics/mini_cheetah_kinematics.h"
#include "drift/utils/type_def.h"

using namespace measurement;

typedef std::pair<IMUQueuePtr, std::shared_ptr<std::mutex>> IMUQueuePair; /**<
Pair of IMUQueuePtr and IMUQueue mutex. */

typedef std::pair<VelocityQueuePtr, std::shared_ptr<std::mutex>>
    VelocityQueuePair; /**< Pair of VelocityQueuePtr and VelocityQueue mutex. */

typedef std::pair<AngularVelocityQueuePtr, std::shared_ptr<std::mutex>>
    AngularVelocityQueuePair; /**< Pair of AngularVelocityQueuePtr and
                                 VelocityQueue mutex. */

typedef std::pair<OdomQueuePtr, std::shared_ptr<std::mutex>> OdomQueuePair; /**<
Pair of OdomQueuePtr and OdomQueue mutex. */

// Legged kinematics sync
typedef std::pair<LeggedKinQueuePtr, std::shared_ptr<std::mutex>>
    LeggedKinQueuePair; /**< Pair of LeggedKinQueuePtr and LeggedKinQueue mutex.
                         */
typedef message_filters::Subscriber<custom_sensor_msgs::ContactArray>
    ContactMsgFilterT; /**< Message filter for contact messages. */
typedef message_filters::Subscriber<sensor_msgs::JointState>
    JointStateMsgFilterT; /**< Message filter for joint state messages. */
typedef std::shared_ptr<
    message_filters::Subscriber<custom_sensor_msgs::ContactArray>>
    ContactMsgFilterTPtr; /**< Pointer to the ContactMsgFilterT. */
typedef std::shared_ptr<message_filters::Subscriber<sensor_msgs::JointState>>
    JointStateMsgFilterTPtr; /**< Pointer to the JointStateMsgFilterT. */
typedef message_filters::sync_policies::ApproximateTime<
    custom_sensor_msgs::ContactArray, sensor_msgs::JointState>
    LegKinSyncPolicy; /**< Sync policy for legged kinematics. */
typedef std::shared_ptr<message_filters::Synchronizer<LegKinSyncPolicy>>
    LegKinSyncPtr; /**< Pointer to the LegKinSyncPolicy. */

// IMU sync
typedef message_filters::Subscriber<sensor_msgs::Imu>
    IMUMsgFilterT; /**< Message filter for IMU messages. */
typedef message_filters::Subscriber<geometry_msgs::Vector3Stamped>
    IMUOffsetMsgFilterT; /**< Message filter for IMU offset messages. */
typedef std::shared_ptr<IMUMsgFilterT>
    IMUMsgFilterTPtr; /**< Pointer to the IMUMsgFilterT. */
typedef std::shared_ptr<IMUOffsetMsgFilterT>
    IMUOffsetMsgFilterTPtr; /**< Pointer to the IMUOffsetMsgFilterT. */
typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Imu, geometry_msgs::Vector3Stamped>
    IMUSyncPolicy; /**< Sync policy for IMU. */
typedef std::shared_ptr<message_filters::Synchronizer<IMUSyncPolicy>>
    IMUSyncPtr; /**< Pointer to the IMUSyncPolicy. */

namespace ros_wrapper {
/**
 * @class ROSSubscriber
 * @brief ROS subscriber class, which subscribes to the ROS topics and stores
 * the measurements in the queue.
 *
 */
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

  /// @name ROS subscriber adders
  /// TODO: Detail what message type each subscribers subscribe to.
  /**
   * @brief Add an IMU subscriber to the given topic and return a queue pair.
   * The queue pair contains the queue and the mutex for the queue. The queue
   * stores the IMU measurements and the mutex is used to protect the queue.
   *
   *
   * @param[in] topic_name IMU topic name
   * @return IMUQueuePair IMU queue pair
   */
  IMUQueuePair AddIMUSubscriber(const std::string topic_name);

  /**
   * @brief Add an IMU subscriber for Fetch to the given topic and return a
   * queue pair. The queue pair contains the queue and the mutex for the queue.
   * The queue stores the IMU measurements and the mutex is used to protect the
   * queue.
   *
   * @param[in] imu_topic_name IMU topic name
   * @param[in] offset_topic_name imu offset topic name (imu bias)
   * @return IMUQueuePair IMU queue pair
   */
  IMUQueuePair AddFetchIMUSubscriber(const std::string imu_topic_name,
                                     const std::string offset_topic_name);

  /**
   * @brief Add velocity subscriber to the given topic and return a queue
   * pair.The queue pair contains the queue and the mutex for the queue. The
   * queue stores the velocity measurements and the mutex is used to protect the
   * queue.
   *
   * @param[in] topic_name velocity topic name
   * @return VelocityQueuePair velocity queue pair
   */
  VelocityQueuePair AddVelocitySubscriber(const std::string topic_name);

  /**
   * @brief Add velocity with covariance subscriber to the given topic and
   * return a queue pair.The queue pair contains the queue and the mutex for the
   * queue. The queue stores the velocity measurements and the mutex is used to
   * protect the queue.
   *
   * @param[in] topic_name velocity topic name
   * @return VelocityQueuePair velocity queue pair
   */
  VelocityQueuePair AddVelocityWithCovarianceSubscriber(
      const std::string topic_name);

  /**
   * @brief Add differential drive velocity subscriber
   *
   * @param[in] topic_name differential drive velocity topic name
   * @param[in] wheel_radius radius of the wheel in meters
   * @return A pair of a pointer to the velocity queue and a mutex to the queue
   */
  VelocityQueuePair AddDifferentialDriveVelocitySubscriber(
      const std::string topic_name, double wheel_radius);

  /**
   * @brief Add differential drive velocity subscriber
   *
   * @param[in] topic_name differential drive velocity topic name
   * @param[in] wheel_radius radius of the wheel in meters
   * @param[in] track_width distance between the two wheels in meters
   * @return A tuple of velocity queue pointer, velocity queue mutex, angular
   * velocity queue pointer, and annular velocity queue mutex
   */
  std::tuple<VelocityQueuePtr, std::shared_ptr<std::mutex>,
             AngularVelocityQueuePtr, std::shared_ptr<std::mutex>>
  AddDifferentialDriveVelocitySubscriber(const std::string topic_name,
                                         double wheel_radius,
                                         double track_width);
  /**
   * @brief
   *
   */
  VelocityQueuePair AddOdom2VelocityCallback(
      const std::string topic_name,
      const std::vector<double>& translation_odomsrc2body,
      const std::vector<double>& rotation_odomsrc2body);

  /**
   * @brief Add differential drive linear velocity subscriber (2 driving wheels)
   * for Fetch to the given topic and return a queue pair. The queue pair
   * contains the queue and the mutex for the queue. The queue stores the
   * velocity measurements and the mutex is used to protect the queue.
   *
   * @param[in] topic_name differential drive velocity topic name
   * @param[in] wheel_radius wheel radius
   * @return VelocityQueuePair velocity queue pair
   */
  VelocityQueuePair AddDifferentialDriveLinearVelocitySubscriber_Fetch(
      const std::string topic_name, double wheel_radius);

  /**
   * @brief Add differential drive velocity (linear + angular) subscriber (2
   * driving wheels) for Fetch to the given topic and return a queue pair. The
   * queue pair contains the queue and the mutex for the queue. The queue stores
   * the velocity measurements and the mutex is used to protect the queue.
   *
   * @param[in] topic_name differential drive velocity topic name
   * @param[in] wheel_radius wheel radius
   * @param[in] track_width track width
   * @return VelocityQueuePair velocity queue pair
   */
  std::tuple<VelocityQueuePtr, std::shared_ptr<std::mutex>,
             AngularVelocityQueuePtr, std::shared_ptr<std::mutex>>
  AddDifferentialDriveVelocitySubscriber_Fetch(const std::string topic_name,
                                               double wheel_radius,
                                               double track_width);

  /**
   * @brief Add differential drive velocity subscriber for Mini Cheetah to the
   * given topic and return a queue pair. The queue pair contains the queue and
   * the mutex for the queue. The queue stores the velocity measurements and the
   * mutex is used to protect the queue.
   *
   * @param[in] contact_topic_name differential drive velocity topic name
   * @param[in] encoder_topic_name encoder topic name
   * @return VelocityQueuePair velocity queue pair
   */
  LeggedKinQueuePair AddMiniCheetahKinematicsSubscriber(
      const std::string contact_topic_name,
      const std::string encoder_topic_name);


  /// @}

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
      const boost::shared_ptr<const geometry_msgs::TwistStamped>& vel_msg,
      const std::shared_ptr<std::mutex>& mutex, VelocityQueuePtr& vel_queue);

  /**
   * @brief Velocity callback function
   *
   * @param[in] vel_msg: velocity with covariance message
   * @param[in] mutex: mutex for the buffer queue
   * @param[in] vel_queue: pointer to the buffer queue
   */
  void VelocityWithCovarianceCallback(
      const boost::shared_ptr<const geometry_msgs::TwistWithCovarianceStamped>& vel_msg,
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
      const std::shared_ptr<std::mutex>& mutex, VelocityQueuePtr& vel_queue,
      double wheel_radius);

  /**
   * @brief Differential encoder to both linear velocity and angular velocity
   *
   * @param[in] encoder_msg: latest encoder message
   * @param[in] vel_mutex: mutex for the linear velocity buffer queue
   * @param[in] ang_vel_mutex: mutex for the angular velocity buffer queue
   * @param[in] vel_queue: pointer to the linear velocity buffer queue
   * @param[in] ang_vel_queue: pointer to the angular velocity buffer queue
   * @param[in] wheel_radius: radius of the wheel in meters
   * @param[in] track_width: distance between the two wheels in meters
   */
  void DifferentialEncoder2VelocityCallback(
      const boost::shared_ptr<const sensor_msgs::JointState>& encoder_msg,
      const std::shared_ptr<std::mutex>& vel_mutex,
      const std::shared_ptr<std::mutex>& ang_vel_mutex,
      VelocityQueuePtr& vel_queue, AngularVelocityQueuePtr& ang_vel_queue,
      double wheel_radius, double track_width);

  /**
   * @brief Differential encoder to linear velocity callback function (Fetch
   * version, 2 driving wheels)
   *
   * @param encoder_msg: encoder message
   * @param mutex: mutex for the buffer queue
   * @param vel_queue: pointer to the buffer queue
   * @param wheel_radius: wheel radius
   * @param track_width: track width
   */
  void DifferentialEncoder2LinearVelocityCallback_Fetch(
      const boost::shared_ptr<const sensor_msgs::JointState>& encoder_msg,
      const std::shared_ptr<std::mutex>& vel_mutex, VelocityQueuePtr& vel_queue,
      double wheel_radius);

  /**
   * @brief Differential encoder to both linear velocity and angular velocity
   * callback function (Fetch version, 2 driving wheels)
   *
   * @param encoder_msg: encoder message
   * @param vel_mutex: mutex for the linear velocity buffer queue
   * @param ang_vel_mutex: mutex for the linear velocity buffer queue
   * @param vel_queue: pointer to the linear velocity buffer queue
   * @param ang_vel_queue: pointer to the linear velocity buffer queue
   * @param wheel_radius: radius of the wheel in meters
   * @param track_width: distance between the two wheels in meters
   */
  void DifferentialEncoder2VelocityCallback_Fetch(
      const boost::shared_ptr<const sensor_msgs::JointState>& encoder_msg,
      const std::shared_ptr<std::mutex>& vel_mutex,
      const std::shared_ptr<std::mutex>& ang_vel_mutex,
      VelocityQueuePtr& vel_queue, AngularVelocityQueuePtr& ang_vel_queue,
      double wheel_radius, double track_width);

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
      const std::shared_ptr<std::mutex>& mutex, LeggedKinQueuePtr& kin_queue);

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

  /**
   * @brief odometry to velocity callback function
   *
   * @param odom_msg: odometry message
   * @param vel_mutex: mutex for the buffer queue
   * @param vel_queue: pointer to the buffer queue
   * @param odom_src_id: odometry source id, given by the adder
   *
   */
  void Odom2VelocityCallback(
      const boost::shared_ptr<const nav_msgs::Odometry>& odom_msg,
      const std::shared_ptr<std::mutex>& vel_mutex, VelocityQueuePtr& vel_queue,
      int odom_src_id);

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
  std::vector<AngularVelocityQueuePtr>
      ang_vel_queue_list_;    // List of angular velocity queue pointers
  std::vector<LeggedKinQueuePtr>
      kin_queue_list_;    // List of kinematics queue pointers
  std::vector<LegKinSyncPtr> leg_kin_sync_list_;
  std::vector<IMUSyncPtr> imu_sync_list_;
  std::vector<std::shared_ptr<std::mutex>> mutex_list_;    // List of mutexes
  std::unordered_map<int, OdomMeasurementPtr>
      prev_odom_map_;                   // odom_src_id -> prev_odom_measurement
  Eigen::Matrix4d odom_src_to_body_;    // Camera to body transformation matrix


  bool thread_started_;    // Flag of the thread started, true for started,
                           // false for not started
  std::thread subscribing_thread_;

  ros::MultiThreadedSpinner spinner_;

  int odom_src_id_ = 0;    // Keep track of the odom source id, start from 0 and
                           // increment by 1 for each new odom source
};

}    // namespace ros_wrapper

#endif
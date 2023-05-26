/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   ros_publisher.h
 *  @author Tingjun Li
 *  @brief  Header file for ROS publisher class
 *  @date   May 16, 2023
 **/

#ifndef ROS_COMMUNICATION_ROS_PUBLISHER_H
#define ROS_COMMUNICATION_ROS_PUBLISHER_H

#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <nav_msgs/Path.h>
#include "boost/bind.hpp"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "ros/ros.h"
#include "yaml-cpp/yaml.h"

#include "drift/state/robot_state.h"
#include "drift/utils/type_def.h"

using namespace state;

namespace ros_wrapper {
/**
 * @class ROSPublisher
 *
 * @brief ROS publisher class, which publishes robot state to ROS topics
 * including pose and path.
 *
 */
class ROSPublisher {
 public:
  /// @name Constructors
  /// @{
  /**
   * @brief Construct a new ROS Publisher object
   *
   * @param[in] nh: ROS node handle
   * @param[in] robot_state_queue: Robot state queue
   * @param[in] robot_state_queue_mutex: Robot state queue mutex
   */
  ROSPublisher(ros::NodeHandle* nh, RobotStateQueuePtr& robot_state_queue,
               std::shared_ptr<std::mutex> robot_state_queue_mutex,
               bool enable_slip_publisher_ = false);
  /**
   * @brief Construct a new ROS Publisher object with configuration file
   *
   * @param[in] nh: ROS node handle
   * @param[in] robot_state_queue: Robot state queue
   * @param[in] robot_state_queue_mutex: Robot state queue mutex
   * @param[in] config_file: Configuration file
   */
  ROSPublisher(ros::NodeHandle* nh, RobotStateQueuePtr& robot_state_queue,
               std::shared_ptr<std::mutex> robot_state_queue_mutex,
               std::string config_file);

  /// @}

  /// @name Destructors
  /// @{
  /**
   * @brief Destroy the ROSPublisher object
   *
   */
  ~ROSPublisher();
  /// @}

  /**
   * @brief Start publishing thread for pose and path publishers
   *
   */
  void StartPublishingThread();

 private:
  ros::NodeHandle* nh_;            // ROS node handle
  RobotStateQueuePtr
      robot_state_queue_ptr_;      // Pointer to the robot state queue
  std::shared_ptr<std::mutex>
      robot_state_queue_mutex_;    // Pointer to the robot state queue mutex

  bool thread_started_;            // Flag for thread started

  ros::Publisher pose_pub_;        // Pose publisher
  std::string
      pose_frame_;           // The name of a frame which poses are published in
  uint32_t pose_seq_ = 0;    // Sequence number for pose publisher
  double pose_publish_rate_;              // Pose publishing rate
  std::thread pose_publishing_thread_;    // Thread for pose publishing

  ros::Publisher path_pub_;               // Path publisher
  uint32_t path_seq_ = 0;                 // Sequence number for path publisher
  double path_publish_rate_;              // Path publishing rate
  std::thread path_publishing_thread_;    // Thread for path publishing

  int pose_skip_;    // Number of poses to skip while generating a path message

  std::array<float, 3> first_pose_;    // Initial pose of the robot
  std::vector<geometry_msgs::PoseStamped>
      poses_;                          // Path message in ROS, a list of poses
  std::mutex poses_mutex_;             // mutex for the path

  bool enable_slip_publisher_;         // Flag for slip publisher

  /// @name Publishing methods
  /// @{
  /**
   * @brief A thread for publishing path messages
   */
  void PathPublishingThread();

  /**
   * @brief Publish the path message
   */
  void PathPublish();

  /**
   * @brief A thread for publishing pose messages
   */
  void PosePublishingThread();

  /**
   * @brief Publish a pose message
   */
  void PosePublish();

  /**
   * @brief Publish slip velocity
   */
  void SlipPublish();

  /**
   * @brief Publish slip flag
   */
  void SlipFlagPublish();

  /// @}
};


}    // namespace ros_wrapper

#endif
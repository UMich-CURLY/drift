/* ----------------------------------------------------------------------------
 * Copyright 2022, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   ros_publisher.h
 *  @author Wenzhe Tong, Tingjun Li
 *  @brief  Header file for ROS2 publisher class
 *  @date   Feburary 16, 2023
 **/

#ifndef ROS2_COMMUNICATION_ROS_PUBLISHER_H
#define ROS2_COMMUNICATION_ROS_PUBLISHER_H

#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <nav_msgs/msg/path.hpp>
#include "boost/bind.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include "state/robot_state.h"

typedef std::queue<std::shared_ptr<RobotState>> RobotStateQueue;
typedef std::shared_ptr<RobotStateQueue> RobotStateQueuePtr;

namespace ros_wrapper {
class ROSPublisher {
 public:
  /// @name Constructors
  /// @{
  /**
   * @brief Construct a new ROS Publisher object
   *
   * @param[in] node: ROS2 node pointer
   * @param[in] robot_state_queue: Robot state queue
   * @param[in] robot_state_queue_mutex: Robot state queue mutex
   */
  ROSPublisher(rclcpp::Node::SharedPtr node,
               RobotStateQueuePtr& robot_state_queue,
               std::shared_ptr<std::mutex> robot_state_queue_mutex);
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
  rclcpp::Node::SharedPtr node_;    // ROS node handle
  RobotStateQueuePtr
      robot_state_queue_ptr_;    // Pointer to the robot state queue
  std::shared_ptr<std::mutex>
      robot_state_queue_mutex_;    // Pointer to the robot state queue mutex

  bool thread_started_;    // Flag for thread started

  rclcpp::Publisher<geometry_msgs::PoseWithCovarianceStamped>::SharedPtr
      pose_pub_;    // Pose publisher
  std::string
      pose_frame_;           // The name of a frame which poses are published in
  uint32_t pose_seq_ = 0;    // Sequence number for pose publisher
  double pose_publish_rate_;              // Pose publishing rate
  std::thread pose_publishing_thread_;    // Thread for pose publishing

  rclcpp::Publisher<nav_msgs::Path>::SharedPtr path_pub_;    // Path publisher
  uint32_t path_seq_ = 0;                 // Sequence number for path publisher
  double path_publish_rate_;              // Path publishing rate
  std::thread path_publishing_thread_;    // Thread for path publishing

  int pose_skip_;    // Number of poses to skip while generating a path message

  std::array<float, 3> first_pose_;    // Initial pose of the robot
  std::vector<geometry_msgs::PoseStamped>
      poses_;                 // Path message in ROS, a list of poses
  std::mutex poses_mutex_;    // mutex for the path

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
  /// @}
};


}    // namespace ros_wrapper

#endif
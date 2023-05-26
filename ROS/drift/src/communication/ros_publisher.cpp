/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   ros_publisher.cpp
 *  @author Tingjun Li
 *  @brief  Source file for ROS publisher class
 *  @date   May 16, 2023
 **/

#include "communication/ros_publisher.h"


namespace ros_wrapper {
ROSPublisher::ROSPublisher(ros::NodeHandle* nh,
                           RobotStateQueuePtr& robot_state_queue_ptr,
                           std::shared_ptr<std::mutex> robot_state_queue_mutex,
                           bool enable_slip_publisher_)
    : nh_(nh),
      robot_state_queue_ptr_(robot_state_queue_ptr),
      robot_state_queue_mutex_(robot_state_queue_mutex),
      thread_started_(false) {
  std::string pose_topic = "/robot/inekf_estimation/pose";
  std::string path_topic = "/robot/inekf_estimation/path";
  pose_frame_ = "/odom";
  pose_publish_rate_ = 1000;    // Hz
  path_publish_rate_ = 10;      // Hz

  first_pose_ = {0, 0, 0};

  std::cout << "pose_topic: " << pose_topic << ", path_topic: " << path_topic
            << std::endl;
  std::cout << "path publish rate: " << path_publish_rate_ << std::endl;

  pose_pub_ = nh_->advertise<geometry_msgs::PoseWithCovarianceStamped>(
      pose_topic, 1000);
  path_pub_ = nh_->advertise<nav_msgs::Path>(path_topic, 1000);
}

ROSPublisher::ROSPublisher(ros::NodeHandle* nh,
                           RobotStateQueuePtr& robot_state_queue_ptr,
                           std::shared_ptr<std::mutex> robot_state_queue_mutex,
                           std::string config_file)
    : nh_(nh),
      robot_state_queue_ptr_(robot_state_queue_ptr),
      robot_state_queue_mutex_(robot_state_queue_mutex),
      thread_started_(false) {
  YAML::Node config = YAML::LoadFile(config_file);
  std::string pose_topic
      = config["publishers"]["pose_publish_topic"].as<std::string>();
  std::string path_topic
      = config["publishers"]["path_publish_topic"].as<std::string>();
  pose_frame_ = config["publishers"]["pose_frame"].as<std::string>();

  pose_publish_rate_ = config["publishers"]["pose_publish_rate"].as<double>();
  path_publish_rate_ = config["publishers"]["path_publish_rate"].as<double>();

  enable_slip_publisher_
      = config["publishers"]["enable_slip_publisher"]
            ? config["publishers"]["enable_slip_publisher"].as<bool>()
            : false;

  first_pose_ = {0, 0, 0};

  std::cout << "pose_topic: " << pose_topic << ", path_topic: " << path_topic
            << std::endl;
  std::cout << "path publish rate: " << path_publish_rate_ << std::endl;

  pose_pub_ = nh_->advertise<geometry_msgs::PoseWithCovarianceStamped>(
      pose_topic, 1000);
  path_pub_ = nh_->advertise<nav_msgs::Path>(path_topic, 1000);
}

ROSPublisher::~ROSPublisher() {
  if (thread_started_ == true) {
    pose_publishing_thread_.join();
    path_publishing_thread_.join();
  }
  poses_.clear();
}

void ROSPublisher::StartPublishingThread() {
  std::cout << "Starting publishing thread" << std::endl;
  this->pose_publishing_thread_
      = std::thread([this] { this->PosePublishingThread(); });
  this->path_publishing_thread_
      = std::thread([this] { this->PathPublishingThread(); });

  thread_started_ = true;
}

// Publishes pose
void ROSPublisher::PosePublish() {
  if (robot_state_queue_ptr_->empty()) {
    return;
  }

  // Get the first pose
  robot_state_queue_mutex_.get()->lock();
  const std::shared_ptr<RobotState> state_ptr = robot_state_queue_ptr_->front();
  robot_state_queue_ptr_->pop();
  robot_state_queue_mutex_.get()->unlock();

  const RobotState& state = *state_ptr.get();

  geometry_msgs::PoseWithCovarianceStamped pose_msg;
  // Header msg
  pose_msg.header.seq = pose_seq_;
  pose_msg.header.stamp = ros::Time().fromSec(state.get_time());
  pose_msg.header.frame_id = pose_frame_;

  // Pose msg
  pose_msg.pose.pose.position.x
      = state.get_world_position()(0) - first_pose_[0];
  pose_msg.pose.pose.position.y
      = state.get_world_position()(1) - first_pose_[1];
  pose_msg.pose.pose.position.z
      = state.get_world_position()(2) - first_pose_[2];

  Eigen::Quaterniond quat(state.get_world_rotation());
  pose_msg.pose.pose.orientation.w = quat.w();
  pose_msg.pose.pose.orientation.x = quat.x();
  pose_msg.pose.pose.orientation.y = quat.y();
  pose_msg.pose.pose.orientation.z = quat.z();

  // Covariance msg
  auto& cov = state.get_P();
  // Get the 6x6 covariance matrix in the following order:
  // (x, y, z, rotation about X axis, rotation about Y axis, rotation
  // about Z axis)
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      pose_msg.pose.covariance[i * 6 + j] = cov(i, j);
    }
  }

  pose_pub_.publish(pose_msg);
  pose_seq_++;

  int pose_skip = pose_publish_rate_
                  / path_publish_rate_;    // Pose publish rate must be faster
                                           // than path publish rate
  if (int(pose_seq_) % pose_skip == 0) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = pose_msg.header;
    pose_stamped.pose = pose_msg.pose.pose;

    std::lock_guard<std::mutex> lock(poses_mutex_);
    poses_.push_back(pose_stamped);
  }
}

// Pose publishing thread
void ROSPublisher::PosePublishingThread() {
  // Loop and publish data
  ros::Rate loop_rate(pose_publish_rate_);
  while (ros::ok()) {
    if (enable_slip_publisher_) {
      SlipPublish();
      SlipFlagPublish();
    }
    PosePublish();
    loop_rate.sleep();
  }
}

// Publishes path
void ROSPublisher::PathPublish() {
  std::lock_guard<std::mutex> lock(poses_mutex_);
  if (poses_.size() == 0) {
    return;
  }
  nav_msgs::Path path_msg;
  path_msg.header.seq = path_seq_;
  path_msg.header.stamp = poses_.back().header.stamp;
  // path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = pose_frame_;
  path_msg.poses = poses_;

  path_pub_.publish(path_msg);
  path_seq_++;
}

// Path publishing thread
void ROSPublisher::PathPublishingThread() {
  // Loop and publish data
  ros::Rate loop_rate(path_publish_rate_);
  while (ros::ok()) {
    PathPublish();
    loop_rate.sleep();
  }
}

void ROSPublisher::SlipPublish() {
  // if (robot_state_queue_ptr_->empty()) {
  //   return;
  // }
  // robot_state_queue_mutex_.get()->lock();
  // const std::shared_ptr<RobotState> state_ptr =
  // robot_state_queue_ptr_->front(); robot_state_queue_mutex_.get()->unlock();
  // const RobotState& state = *state_ptr.get();

  // geometry_msgs::Vector3Stamped slip_msg;
  // slip_msg.header.stamp = ros::Time().fromSec(state.get_time());
  // slip_msg.header.frame_id = pose_frame_;
  // auto dist = state.get_aug_state(state.dimX() - 1);
  // slip_msg.vector.x = dist(0);
  // slip_msg.vector.y = dist(1);
  // slip_msg.vector.z = dist(2);
  // slip_pub_.publish(slip_msg);
}

void ROSPublisher::SlipFlagPublish() {
  // Get state
  // if (robot_state_queue_ptr_->empty()) {
  //   return;
  // }
  // robot_state_queue_mutex_.get()->lock();
  // const std::shared_ptr<RobotState> state_ptr =
  // robot_state_queue_ptr_->front(); robot_state_queue_mutex_.get()->unlock();
  // const RobotState& state = *state_ptr.get();

  // geometry_msgs::Vector3Stamped slip_flag_msg;
  // slip_flag_msg.header.stamp = ros::Time().fromSec(state.get_time());
  // slip_flag_msg.header.frame_id = pose_frame_;
  // int slip_flag = state.get_slip_flag();
  // slip_flag_msg.vector.x = (double) slip_flag;
  // slip_flag_msg.vector.y = (double) slip_flag;
  // slip_flag_msg.vector.z = (double) slip_flag;
  // slip_flag_pub_.publish(slip_flag_msg);
}

}    // namespace ros_wrapper
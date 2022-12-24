#include "communication/ros_publisher.h"

namespace ros_wrapper {
ROSPublisher::ROSPublisher(ros::NodeHandle* nh,
                           RobotStateQueuePtr& robot_state_queue_ptr,
                           std::shared_ptr<std::mutex> robot_state_queue_mutex)
    : nh_(nh),
      robot_sate_queue_ptr_(robot_state_queue_ptr),
      robot_state_queue_(*robot_state_queue_ptr.get()),
      robot_state_queue_mutex_(robot_state_queue_mutex),
      thread_started_(false) {
  std::string pose_topic;
  std::string path_topic;
  nh_->param<std::string>("/settings/pose_topic", pose_topic,
                          "/robot/inekf_estimation/pose");
  nh_->param<std::string>("/settings/map_frame_id", pose_frame_, "/odom");
  nh->param<std::string>("/settings/path_topic", path_topic,
                         "/robot/inekf_estimation/path");
  nh_->param<double>("/settings/pose_publish_rate", pose_publish_rate_, 1000);
  nh_->param<double>("/settings/path_publish_rate", path_publish_rate_, 1000);
  nh_->param<int>("/settings/pose_skip", pose_skip_, 1);
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

void ROSPublisher::start_publishing_thread() {
  std::cout << "start publishing thread" << std::endl;
  this->pose_publishing_thread_
      = std::thread([this] { this->posePublishingThread(); });
  this->path_publishing_thread_
      = std::thread([this] { this->pathPublishingThread(); });

  thread_started_ = true;
}

// Publishes pose
void ROSPublisher::posePublish() {
  if (robot_state_queue_.empty()) {
    // std::cout << "pose queue is empty" << std::endl;
    return;
  }

  // Get the first pose
  robot_state_queue_mutex_.get()->lock();
  const std::shared_ptr<RobotState> state_ptr = robot_state_queue_.front();
  robot_state_queue_.pop();
  robot_state_queue_mutex_.get()->unlock();

  const RobotState& state = *state_ptr.get();

  geometry_msgs::PoseWithCovarianceStamped pose_msg;
  // Header msg
  pose_msg.header.seq = pose_seq_;
  int sec = int(state.get_time());
  int nsec = (state.get_time() - sec) * 1e9;
  pose_msg.header.stamp.sec = sec;
  pose_msg.header.stamp.sec = nsec;
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

  std::cout << "pose publishing: " << pose_msg.pose.pose.position.x << ", "
            << pose_msg.pose.pose.position.y << ", "
            << pose_msg.pose.pose.position.z << std::endl;
  pose_pub_.publish(pose_msg);
  pose_seq_++;

  if (int(pose_seq_) % pose_skip_ == 0) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = pose_msg.header;
    pose_stamped.pose = pose_msg.pose.pose;

    std::lock_guard<std::mutex> lock(poses_mutex_);
    poses_.push_back(pose_stamped);
  }
}

// Pose publishing thread
void ROSPublisher::posePublishingThread() {
  // Loop and publish data
  ros::Rate loop_rate(pose_publish_rate_);
  while (ros::ok()) {
    posePublish();
    loop_rate.sleep();
  }
}

// Publishes path
void ROSPublisher::pathPublish() {
  std::lock_guard<std::mutex> lock(poses_mutex_);
  if (poses_.size() == 0) {
    // std::cout << "path is empty" << std::endl;
    return;
  }
  nav_msgs::Path path_msg;
  path_msg.header.seq = path_seq_;
  path_msg.header.stamp = poses_.back().header.stamp;
  path_msg.header.frame_id = pose_frame_;
  path_msg.poses = poses_;
  // std::cout<<"publishing current path:
  // "<<path_msg.poses.back().pose.position.x<<",
  // "<<path_msg.poses.back().pose.position.y<<",
  // "<<path_msg.poses.back().pose.position.z<<std::endl;

  path_pub_.publish(path_msg);
  path_seq_++;
}

// Path publishing thread
void ROSPublisher::pathPublishingThread() {
  // Loop and publish data
  ros::Rate loop_rate(path_publish_rate_);
  while (ros::ok()) {
    pathPublish();
    loop_rate.sleep();
  }
}

}    // namespace ros_wrapper
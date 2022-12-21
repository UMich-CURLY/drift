#include "communication/ros_publisher.h"

namespace ros_wrapper {
ROSPublisher::ROSPublisher(ros::NodeHandle* nh)
    : nh_(nh), thread_started_(false) {
  nh_.param<std::string>("/settings/pose_topic", pose_topic,
                         "/husky/inekf_estimation/pose");
  nh_.param<std::string>("/settings/map_frame_id", pose_frame_, "/odom");
  nh.param<std::string>("/settings/path_topic", path_topic,
                        "/husky/inekf_estimation/path");
  nh_.param<double>("/settings/publish_rate", publish_rate_, 1000);
  nh_.param<uint32_t>("/settings/pose_skip", pose_skip_, 0);
  first_pose_ = {0, 0, 0};

  std::cout << "pose_topic: " << pose_topic << ", path_topic: " << path_topic
            << std::endl;
  std::cout << "path publish rate: " << publish_rate_ << std::endl;

  pose_pub_ = nh_->advertise<geometry_msgs::PoseWithCovarianceStamped>(
      pose_topic, 1000);
  path_pub_ = n_.advertise<nav_msgs::Path>(path_topic, 10);
  this->pose_publishing_thread_
      = std::thread([this] { this->posePublishingThread(); });
  this->path_publishing_thread_
      = std::thread([this] { this->pathPublishingThread(); });
}

ROSPublisher::~ROSPublisher() {
  if (thread_started_ == true) {
    pose_publishing_thread_.join();
    path_publishing_thread_.join();
  }
  poses_.clear();
}

// Publishes pose
void ROSPublisher::posePublish(const husky_inekf::HuskyState& state_) {
  // std::array<float,3> cur_pose = pose_from_csv_.front();
  // pose_from_csv_.pop();

  geometry_msgs::PoseWithCovarianceStamped pose_msg;
  pose_msg.header.seq = pose_seq_;
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id = pose_frame_;
  pose_msg.pose.pose.position.x = state_.x() - first_pose_[0];
  pose_msg.pose.pose.position.y = state_.y() - first_pose_[1];
  pose_msg.pose.pose.position.z = state_.z() - first_pose_[2];
  pose_msg.pose.pose.orientation.w = state_.getQuaternion().w();
  pose_msg.pose.pose.orientation.x = state_.getQuaternion().x();
  pose_msg.pose.pose.orientation.y = state_.getQuaternion().y();
  pose_msg.pose.pose.orientation.z = state_.getQuaternion().z();
  // std::cout<<"publishing: "<<pose_msg.pose.pose.position.x<<",
  // "<<pose_msg.pose.pose.position.y<<",
  // "<<pose_msg.pose.pose.position.z<<std::endl;
  pose_pub_.publish(pose_msg);
  pose_seq_++;

  if (seq_ % pose_skip_ == 0) {
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
  ros::Rate loop_rate(publish_rate_);
  while (ros::ok()) {
    posePublish();
    loop_rate.sleep();
  }
}

// Publishes path
void ROSPublisher::pathPublish() {
  std::lock_guard<std::mutex> lock(poses_mutex_);
  nav_msgs::Path path_msg;
  path_msg.header.seq = path_seq_;
  path_msg.header.stamp = ros::Time::now();
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
  ros::Rate loop_rate(publish_rate_);
  while (ros::ok()) {
    pathPublish();
    loop_rate.sleep();
  }
}

}    // namespace ros_wrapper
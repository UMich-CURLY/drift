# Other Communication Wrappers
Although we provided a ROS wrapper inside our repository, users can actually write their own communication wrappers according to their needs. The only requirement is that the wrapper should be able to send and receive messages to and from the robot or simulator. In this tutorial, we will show you how to write a communication wrapper. The wrapper includes three parts: message types, communication subscriber, and communication publisher. It is relatively simple once you understan the framework. 

### Step 1: Create custom message types if necessary
Although most of the time, you can use the message types provided by the simulator or robot, sometimes you may need to create your own message types. For example, in our case, we need to create `contact` related messages for the MIT mini-cheetah robot. In this case, we created the new message types by following the [ROS tutorial](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv) and put them under `drift/ROS/drift/msg` folder. After creating the message type, the `./build_ros.sh` script will automatically build the message types. The specific lines of message builder is shown below:
```bash
MSG_NAMESPACE=custom_sensor_msgs
MSG_PATH=./ROS/drift/msg
MSG_HEADER_OUTPUT_PATH=./ROS/drift/include/$MSG_NAMESPACE/

for file in $MSG_PATH/*
do
	if [[ -a $file ]]
	then
		rosrun gencpp gen_cpp.py -p $MSG_NAMESPACE -o $MSG_HEADER_OUTPUT_PATH -e /opt/ros/noetic/share/gencpp "$file" -I std_msgs:/opt/ros/noetic/share/std_msgs/msg -I $MSG_NAMESPACE:$MSG_PATH
	fi
done
```

If you need to use other communication methods, please refer to corresponding documentation on how to build custom message types.

### Step 2: Create a new communication subscriber class
A communication subscriber mainly acts like an interface between robot/simulator messages and filter defined measurement types. The subscriber should be able to receive messages from the robot/simulator, and push them into corresponding sensor data buffers. In our filter, we used `std::queue<std::shared_ptr<TYPE>>` to be the sensor data buffer type. For example, when an IMU message comes in, the IMU subscriber should be able to convert the IMU message to `ImuMeasurement<double>` type and push it into a `std::queue<std::shared_ptr<ImuMeasurement<double>>>` data buffer. Because this data buffer is also shared with the filter interface on a different thread, a common `std::mutex` is required to protect the data sharing between threads. The following code snippet shows how we implemented the IMU subscriber in ROS. More examples can be found in `ros_subscriber.cpp`.
```cpp
// Use a function to add a new IMU subscriber
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

// The true function that handles IMU message
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
```
Users can also create their own way in building subscribers. But please make sure that the message type is converted to the measurement type defined in the filter.

### Step 3: Create a new communication publisher class
A publisher class is actually optional. If a user simply wants to have the poses saved into a file, she/he can simply write the pose data from `RobotStateQueue` buffer into a `.txt` file. But having a publisher offers the ability to monitor the path output in realtime. 

The publisher class is much simpler than the subscriber class. This class acts as an interface between filter output and visualizable messages. The filter output is stored in a data buffer of type `RobotStateQueue`, which is `std::queue<std::shared_ptr<RobotState>>`. The publisher class's job is to read data from the buffer and use user prefered communication method to publish the data. In our case with ROS communication, the publisher converts the state estimate message to `geometry_msgs::PoseStamped` type. Because we also want to have a visulizable path shown in RVIZ, the message is also pushed into a `std::vector<geometry_msgs::PoseStamped>` data buffer, which will be constructed into a `nav_msgs::Path` message. Because `RobotState` data buffer is also shared with the filter interface on a different thread, a common `std::mutex` is required to protect the data sharing between threads. The following code snippet shows how we implemented the publisher class in ROS. Header file can be found in `ros_publisher.h`.
```cpp
```cpp
/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   ros_publisher.cpp
 *  @author Tingjun Li
 *  @brief  Source file for ROS publisher class
 *  @date   December 20, 2022
 **/

#include "communication/ros_publisher.h"


namespace ros_wrapper {
ROSPublisher::ROSPublisher(ros::NodeHandle* nh,
                           RobotStateQueuePtr& robot_state_queue_ptr,
                           std::shared_ptr<std::mutex> robot_state_queue_mutex)
    : nh_(nh),
      robot_state_queue_ptr_(robot_state_queue_ptr),
      robot_state_queue_mutex_(robot_state_queue_mutex),
      thread_started_(false) {
  std::string pose_topic;
  std::string path_topic;
  nh_->param<std::string>("/curly_state_est_settings/pose_topic", pose_topic,
                          "/robot/inekf_estimation/pose");
  nh_->param<std::string>("/curly_state_est_settings/map_frame_id", pose_frame_,
                          "/odom");
  nh->param<std::string>("/curly_state_est_settings/path_topic", path_topic,
                         "/robot/inekf_estimation/path");
  nh_->param<double>("/curly_state_est_settings/pose_publish_rate",
                     pose_publish_rate_, 1000);
  nh_->param<double>("/curly_state_est_settings/path_publish_rate",
                     path_publish_rate_, 1000);
  nh_->param<int>("/curly_state_est_settings/pose_skip", pose_skip_, 1);
  first_pose_ = {0, 0, 0};

  std::cout << "pose_topic: " << pose_topic << ", pose publish rate: " << pose_publish_rate << std::endl;
  std::cout << "path_topic: " << path_topic << ", path publish rate: " << path_publish_rate_ << std::endl;

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
  std::cout << "start publishing thread" << std::endl;
  this->pose_publishing_thread_
      = std::thread([this] { this->PosePublishingThread(); });
  this->path_publishing_thread_
      = std::thread([this] { this->PathPublishingThread(); });

  thread_started_ = true;
}

// Publishes pose
void ROSPublisher::PosePublish() {
  if (robot_state_queue_ptr_->empty()) {
    // std::cout << "pose queue is empty" << std::endl;
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

  if (int(pose_seq_) % pose_skip_ == 0) {
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
    PosePublish();
    loop_rate.sleep();
  }
}

// Publishes path
void ROSPublisher::PathPublish() {
  std::lock_guard<std::mutex> lock(poses_mutex_);
  if (poses_.size() == 0) {
    // std::cout << "path is empty" << std::endl;
    return;
  }
  nav_msgs::Path path_msg;
  path_msg.header.seq = path_seq_;
  path_msg.header.stamp = poses_.back().header.stamp;
  // path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = pose_frame_;
  path_msg.poses = poses_;

  path_pub_.publish(pa  th_msg);
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

}    // namespace ros_wrapper
```
Once the above three steps are finished, user may finish up writing the main function as well as the CMakelists.txt file. You are also more than welcome to share your wrapper to our repo by creating a pull request! 

Enjoy!

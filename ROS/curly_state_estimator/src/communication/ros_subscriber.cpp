
#include "communication/ros_subscriber.h"

namespace ros_wrapper {


ROSSubscriber::ROSSubscriber(ros::NodeHandle* nh)
    : nh_(nh), thread_started_(false) {}


ROSSubscriber::~ROSSubscriber() {
  if (thread_started_ == true) {
    subscribing_thread_.join();
  }
  subscriber_list_.clear();
  imu_queue_list_.clear();
}


IMUQueuePtr ROSSubscriber::add_imu_subscriber(const std::string topic_name) {
  // Create a new queue for data buffers
  IMUQueuePtr imu_queue_ptr(new IMUQueue);

  // Initialize a new mutex for this subscriber
  mutex_list_.emplace_back(new std::mutex);

  // Create the subscriber
  subscriber_list_.push_back(nh_->subscribe<sensor_msgs::Imu>(
      topic_name, 1000,
      boost::bind(&ROSSubscriber::imu_call_back, this, _1, mutex_list_.back(),
                  imu_queue_ptr)));

  // Keep the ownership of the data queue in this class
  imu_queue_list_.push_back(imu_queue_ptr);

  return imu_queue_ptr;
}


void ROSSubscriber::start_subscribing_thread() {
  subscribing_thread_ = std::thread([this] { this->ros_spin(); });
  thread_started_ = true;
}

void ROSSubscriber::imu_call_back(
    const boost::shared_ptr<const sensor_msgs::Imu>& imu_msg,
    const std::shared_ptr<std::mutex> mutex, IMUQueuePtr& imu_queue) {
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

  std::lock_guard<std::mutex> lock(*mutex);
  imu_queue->push(imu_measurement);
}


void ROSSubscriber::ros_spin() {
  while (ros::ok()) {
    ros::spinOnce();
  }
}

}    // namespace ros_wrapper

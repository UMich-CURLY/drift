
#include "communication/ros_subscriber.h"

namespace ros_wrapper {


ROSSubscriber::ROSSubscriber(ros::NodeHandle* nh) : nh_(nh) {}


ROSSubscriber::~ROSSubscriber() {
  subscribing_thread_.join();
  subscriber_list_.clear();
  imu_queue_list_.clear();
}


IMUQueuePtr ROSSubscriber::add_imu_subscriber(const std::string topic_name) {
  IMUQueuePtr imu_queue_ptr(new IMUQueue);

  // std::mutex new_mutex;
  mutex_list_.emplace_back(new std::mutex);

  subscriber_list_.push_back(nh_->subscribe<sensor_msgs::Imu>(
      topic_name, 1000,
      boost::bind(&ROSSubscriber::imu_call_back, this, _1, imu_queue_ptr,
                  mutex_list_.back())));

  imu_queue_list_.push_back(imu_queue_ptr);

  return imu_queue_ptr;
}

void ROSSubscriber::imu_call_back(
    const boost::shared_ptr<const sensor_msgs::Imu>& imu_msg,
    IMUQueuePtr& imu_queue, std::shared_ptr<std::mutex> mutex) {
  // create a temporate imu measurement object
  std::shared_ptr<ImuMeasurement<double>> imu_measurement(
      new ImuMeasurement<double>);


  imu_measurement->set_lin_acc(test_idx, test_idx, test_idx);
  test_idx += 1;

  // std::lock_guard<std::mutex> lock(*mutex);
  imu_queue->push(imu_measurement);
}


void ROSSubscriber::ros_spin() {
  while (ros::ok()) {
    ros::spinOnce();
  }
}

}    // namespace ros_wrapper


#include "communication/ros_subscriber.h"

namespace ros_wrapper {


ROSSubscriber::ROSSubscriber(ros::NodeHandle* nh) : nh_(nh) {}


ROSSubscriber::~ROSSubscriber() {
  subscribing_thread_.join();
  subscriber_list_.clear();
}


void ROSSubscriber::add_imu_subscriber(const std::string topic_name) {
  subscriber_list_.push_back(
      nh_->subscribe(topic_name, 1000, &ROSSubscriber::imu_call_back, this));

  // subscriber_list_.emplace_back(imu_sub);
}

void ROSSubscriber::imu_call_back(const sensor_msgs::Imu& imu_msg) {
  std::cout << imu_msg.angular_velocity.x << std::endl;
}

void ROSSubscriber::ros_spin() {
  while (ros::ok()) {
    ros::spinOnce();
  }
}

}    // namespace ros_wrapper

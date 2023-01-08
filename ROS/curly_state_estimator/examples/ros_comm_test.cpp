#include <ros/ros.h>
#include <iostream>

#include "communication/ros_publisher.h"
#include "communication/ros_subscriber.h"
#include "state_estimator.h"
using namespace std;


int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_state_est");

  std::cout << "The subscriber is on!" << std::endl;

  // ros handle handles the start/shutdown for us
  ros::NodeHandle nh;

  // Subscriber:
  ros_wrapper::ROSSubscriber ros_sub(&nh);

  auto qimu_and_mutex = ros_sub.add_imu_subscriber("/gx5_1/imu/data");
  auto qimu = qimu_and_mutex.first;
  auto qimu_mutex = qimu_and_mutex.second;

  auto qv_and_mutex
      = ros_sub.add_differential_drive_velocity_subscriber("/joint_states");
  auto qv = qv_and_mutex.first;
  auto qv_mutex = qv_and_mutex.second;

  ros_sub.start_subscribing_thread();
  // TODO: Create robot state system -- initialize all system threads


  inekf::ErrorType error_type = RightInvariant;
  StateEstimator state_estimator(error_type);

  // Publisher:
  state_estimator.add_imu_propagation(qimu, qimu_mutex);    // Husky's setting
  state_estimator.add_velocity_correction(qv, qv_mutex);
  RobotStateQueuePtr robot_state_queue_ptr
      = state_estimator.get_robot_state_queue_ptr();

  std::shared_ptr<std::mutex> robot_state_queue_mutex_ptr
      = state_estimator.get_robot_state_queue_mutex_ptr();
  ros_wrapper::ROSPublisher ros_pub(&nh, robot_state_queue_ptr,
                                    robot_state_queue_mutex_ptr);
  ros_pub.start_publishing_thread();

  // Start running
  while (ros::ok()) {
    // Step behavior
    if (state_estimator.enabled()) {
      state_estimator.run_once();
    } else {
      if (state_estimator.biasInitialized()) {
        state_estimator.initStateFromImu();
      } else {
        state_estimator.initBias();
      }
    }
    ros::spinOnce();
  }

  return 0;
}
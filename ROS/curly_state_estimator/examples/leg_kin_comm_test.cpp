#include <ros/ros.h>
#include <iostream>

// #include "communication/ros_publisher.h"
#include "communication/ros_subscriber.h"
// #include "state_estimator.h"
using namespace std;


int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_state_est");

  std::cout << "The subscriber is on!" << std::endl;

  // ros handle handles the start/shutdown for us
  ros::NodeHandle nh;

  // Subscriber:
  ros_wrapper::ROSSubscriber ros_sub(&nh);
  std::cout << "Subscribing to imu channel..." << std::endl;
  auto qimu_and_mutex = ros_sub.AddIMUSubscriber("/Imu");
  auto qimu = qimu_and_mutex.first;
  auto qimu_mutex = qimu_and_mutex.second;

  std::cout << "Subscribing to joint_states and contact channel..."
            << std::endl;
  auto qkin_and_mutex
      = ros_sub.AddMiniCheetahKinematicsSubscriber("/Contacts", "/JointState");
  auto qkin = qkin_and_mutex.first;
  auto qkin_mutex = qkin_and_mutex.second;

  ros_sub.StartSubscribingThread();
  // TODO: Create robot state system -- initialize all system threads


  // inekf::ErrorType error_type = RightInvariant;
  // StateEstimator state_estimator(error_type);

  // Publisher:
  // state_estimator.add_imu_propagation(qimu, qimu_mutex);    // Husky's
  // setting state_estimator.add_velocity_correction(qv, qv_mutex);
  // RobotStateQueuePtr robot_state_queue_ptr
  //     = state_estimator.get_robot_state_queue_ptr();

  // std::shared_ptr<std::mutex> robot_state_queue_mutex_ptr
  //     = state_estimator.get_robot_state_queue_mutex_ptr();
  // ros_wrapper::ROSPublisher ros_pub(&nh, robot_state_queue_ptr,
  //                                   robot_state_queue_mutex_ptr);
  // ros_pub.StartPublishingThread();

  // Start running
  while (ros::ok()) {
    // // Step behavior
    // if (state_estimator.is_enabled()) {
    //   state_estimator.RunOnce();
    // } else {
    //   if (state_estimator.BiasInitialized()) {
    //     state_estimator.InitStateFromImu();
    //   } else {
    //     state_estimator.InitBias();
    //   }
    // }
    ros::spinOnce();
  }

  std::cout << "qimu msg: " << std::endl;
  for (int i = 0; i < 10; ++i) {
    auto qimu_first = qimu->front()->get_lin_acc();
    auto qimu_ang = qimu->front()->get_ang_vel();
    auto qimu_orie = qimu->front()->get_quaternion();
    auto qimu_t = qimu->front()->get_time();
    std::cout << std::setprecision(16) << qimu_t << ", " << qimu_first[0]
              << ", " << qimu_first[1] << ", " << qimu_first[2] << ", "
              << qimu_ang[0] << ", " << qimu_ang[1] << ", " << qimu_ang[2]
              << "," << qimu_orie.w() << "," << qimu_orie.x() << ","
              << qimu_orie.y() << "," << qimu_orie.z() << std::endl;
    qimu->pop();
  }

  std::cout << "qkin contact msg: " << std::endl;
  for (int i = 0; i < 10; ++i) {
    auto qkin_first = qkin->front();
    auto qkin_t = qkin->front()->get_time();
    std::cout << std::setprecision(16) << qkin_t << ", "
              << qkin_first->get_contact(0) << ", "
              << qkin_first->get_contact(1) << ", "
              << qkin_first->get_contact(2) << ", "
              << qkin_first->get_contact(3) << std::endl;
    qkin->pop();
  }

  return 0;
}
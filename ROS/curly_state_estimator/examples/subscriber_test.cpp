#include <ros/ros.h>
#include <iostream>

#include "communication/ros_subscriber.h"

using namespace std;


int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_state_est");
  ros::start();

  // if (argc != 3) {
  //   cerr << endl
  //        << "Usage: rosrun curly_state_estimator robot_state_est "
  //           "path_to_vocabulary "
  //           "path_to_settings"
  //        << endl;
  //   ros::shutdown();
  //   return 1;
  // }

  std::cout << "The subscriber is on!" << std::endl;
  ros::NodeHandle nh;


  ros_wrapper::ROSSubscriber ros_sub(&nh);
  auto q1 = ros_sub.add_imu_subscriber("/gx5_0/imu/data");
  auto q2 = ros_sub.add_imu_subscriber("/gx5_1/imu/data");
  // TODO: Create robot state system -- initialize all system threads

  // Set noise parameters. From husky_estimator:
  // inekf::NoiseParams params;

  while (ros::ok()) {
    // Step behavior

    ros::spinOnce();
  }

  std::cout << "q1 msg: " << std::endl;
  for (int i = 0; i < 10; ++i) {
    auto q1_first = q1->front()->get_lin_acc();
    std::cout << q1_first[0] << ", " << q1_first[1] << ", " << q1_first[2]
              << std::endl;
    q1->pop();
  }

  std::cout << "q2 msg: " << std::endl;
  for (int i = 0; i < 10; ++i) {
    auto q2_first = q2->front()->get_lin_acc();
    std::cout << q2_first[0] << ", " << q2_first[1] << ", " << q2_first[2]
              << std::endl;
    q2->pop();
  }

  // Stop all threads

  ros::shutdown();

  return 0;
}
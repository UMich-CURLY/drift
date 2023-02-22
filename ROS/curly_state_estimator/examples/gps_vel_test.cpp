#include <ros/ros.h>
#include <iostream>

#include "communication/ros_publisher.h"
#include "communication/ros_subscriber.h"

using namespace std;


int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_state_est");

  std::cout << "The subscriber is on!" << std::endl;

  // ros handle handles the start/shutdown for us
  ros::NodeHandle nh;

  ros_wrapper::ROSSubscriber ros_sub(&nh);
  auto q1_and_mutex = ros_sub.AddGPSVelocitySubscriber("/gps/vel");
  auto q1 = q1_and_mutex.first;
  ros_sub.StartSubscribingThread();
  // TODO: Create robot state system -- initialize all system threads

  // block until we stop the ros to print out the value
  while (ros::ok()) {
    // Step behavior

    // ros::spinOnce();
    auto q1_first = q1->front()->get_velocity();
    auto q1_ang = q1->front()->get_angular_velocity();
    auto q1_t = q1->front()->get_time();
    std::cout << std::setprecision(16) << "timestamp: " << q1_t << std::endl
              << "linear velocity: " << q1_first[0] << ", " << q1_first[1]
              << ", " << q1_first[2] << std::endl
              << "angular velocity: " << q1_ang[0] << ", " << q1_ang[1] << ", "
              << q1_ang[2] << std::endl;
    // q1->pop();
  }

  // std::cout << "q1 msg: " << std::endl;
  // for (int i = 0; i < 10; ++i) {
  //   auto q1_first = q1->front()->get_velocity();
  //   auto q1_ang = q1->front()->get_angular_velocity();
  //   auto q1_t = q1->front()->get_time();
  //   std::cout << std::setprecision(16) << "timestamp: " << q1_t << std::endl
  //             << "linear velocity: " << q1_first[0] << ", " << q1_first[1]
  //             << ", " << q1_first[2] << std::endl
  //             << "angular velocity: " << q1_ang[0] << ", " << q1_ang[1] << ",
  //             "
  //             << q1_ang[2] << std::endl;
  //   q1->pop();
  // }


  return 0;
}
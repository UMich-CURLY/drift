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
  auto q2_and_mutex = ros_sub.AddGPSNavSatSubscriber("/gps/fix");
  auto q2 = q2_and_mutex.first;
  ros_sub.StartSubscribingThread();
  // TODO: Create robot state system -- initialize all system threads

  // block until we stop the ros to print out the value
  while (ros::ok()) {
    // Step behavior

    ros::spinOnce();
  }

  std::cout << "q1 msg: " << std::endl;
  for (int i = 0; i < 10; ++i) {
    auto q1_first = q1->front()->get_velocity();
    auto q1_ang = q1->front()->get_angular_velocity();
    auto q1_t = q1->front()->get_time();
    std::cout << std::setprecision(16) << "timestamp: " << q1_t << std::endl
              << "linear velocity: " << q1_first[0] << ", " << q1_first[1]
              << ", " << q1_first[2] << std::endl
              << "angular velocity: " << q1_ang[0] << ", " << q1_ang[1] << ", "
              << q1_ang[2] << std::endl;
    q1->pop();
  }

  auto q2_geo0 = q2->front()->get_geodetic();
  for (int i = 0; i < 10; ++i) {
    auto q2_fix = q2->front()->get_geodetic();
    auto q2_enu = q2->front()->get_enu(q2_geo0[0], q2_geo0[1], q2_geo0[2]);
    auto q2_t = q1->front()->get_time();
    std::cout << std::setprecision(16) << "timestamp: " << q2_t << std::endl
              << "lat, lon, alt (deg): " << q2_fix[0] << ", " << q2_fix[1]
              << ", " << q2_fix[2] << std::endl
              << "east north up (meters): " << q2_enu[0] << ", " << q2_enu[1]
              << ", " << q2_enu[2] << std::endl;
    q1->pop();
  }


  return 0;
}
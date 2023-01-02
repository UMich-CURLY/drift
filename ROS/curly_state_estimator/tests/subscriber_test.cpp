#include <ros/ros.h>
#include <iostream>

#include "communication/ros_subscriber.h"

using namespace std;


int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_state_est");

  std::cout << "The subscriber is on!" << std::endl;

  // ros handle handles the start/shutdown for us
  ros::NodeHandle nh;

  ros_wrapper::ROSSubscriber ros_sub(&nh);
  auto q1 = ros_sub.add_imu_subscriber("/gx5_0/imu/data");
  auto q2 = ros_sub.add_imu_subscriber("/gx5_1/imu/data");
  auto qv = ros_sub.add_differential_drive_velocity_subscriber("/joint_states");
  ros_sub.start_subscribing_thread();
  // TODO: Create robot state system -- initialize all system threads

  // block until we stop the ros to print out the value
  while (ros::ok()) {
    // Step behavior

    // ros::spinOnce();
  }

  std::cout << "q1 msg: " << std::endl;
  for (int i = 0; i < 10; ++i) {
    auto q1_first = q1->front()->get_lin_acc();
    auto q1_ang = q1->front()->get_ang_vel();
    auto q1_orie = q1->front()->get_quaternion();
    auto q1_t = q1->front()->get_time();
    std::cout << std::setprecision(16) << q1_t << ", " << q1_first[0] << ", "
              << q1_first[1] << ", " << q1_first[2] << ", " << q1_ang[0] << ", "
              << q1_ang[1] << ", " << q1_ang[2] << "," << q1_orie.w() << ","
              << q1_orie.x() << "," << q1_orie.y() << "," << q1_orie.z()
              << std::endl;
    q1->pop();
  }

  std::cout << "q2 msg: " << std::endl;
  for (int i = 0; i < 10; ++i) {
    auto q2_first = q2->front()->get_lin_acc();
    auto q2_ang = q2->front()->get_ang_vel();
    auto q2_orie = q2->front()->get_quaternion();
    auto q2_t = q2->front()->get_time();
    std::cout << std::setprecision(16) << q2_t << ", " << q2_first[0] << ", "
              << q2_first[1] << ", " << q2_first[2] << ", " << q2_ang[0] << ", "
              << q2_ang[1] << ", " << q2_ang[2] << "," << q2_orie.w() << ","
              << q2_orie.x() << "," << q2_orie.y() << "," << q2_orie.z()
              << std::endl;
    q2->pop();
  }

  std::cout << "qv msg: " << std::endl;
  for (int i = 0; i < 10; ++i) {
    auto qv_first = qv->front()->get_velocity();
    auto qv_t = qv->front()->get_time();
    std::cout << std::setprecision(16) << qv_t << ", " << qv_first[0] << ", "
              << qv_first[1] << ", " << qv_first[2] << std::endl;
    qv->pop();
  }


  return 0;
}
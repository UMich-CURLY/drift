#include <ros/ros.h>
#include <iostream>

#include "communication/robot_data.h"

using namespace std;


int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_state_est");
  ros::start();

  if (argc != 3) {
    cerr << endl
         << "Usage: rosrun curly_state_estimator robot_state_est "
            "path_to_vocabulary "
            "path_to_settings"
         << endl;
    ros::shutdown();
    return 1;
  }
  ros::NodeHandle nh;

  // TODO: Create robot state system -- initialize all system threads

  // Set noise parameters. From husky_estimator:
  // inekf::NoiseParams params;

  while (ros::ok()) {
    // Step behavior

    ros::spinOnce();
  }

  // Stop all threads

  ros::shutdown();

  return 0;
}
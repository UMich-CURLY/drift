#include <ros/ros.h>
#include <iostream>

using namespace std;


int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_state");
  ros::start();

  if (argc != 3) {
    cerr << endl
         << "Usage: rosrun CSE robot_state path_to_vocabulary path_to_settings"
         << endl;
    ros::shutdown();
    return 1;
  }

  // TODO: Create robot state system -- initialize all system threads

  ros::NodeHandle nodeHandler;
  // ros::Subscriber sub = nodeHandler.subscribe();

  ros::spin();
  // Send some output as log message
  ROS_INFO_STREAM("Hello, ROS!");

  // TODO: Stop all threads

  ros::shutdown();

  return 0;
}
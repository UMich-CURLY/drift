#include <ros/ros.h>
#include <iostream>

#include "communication/ros_publisher.h"
#include "communication/ros_subscriber.h"
#include "state_estimator/state_estimator.h"
using namespace std;


int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_state_est");

  std::cout << "The subscriber is on!" << std::endl;

  // ros handle handles the start/shutdown for us
  ros::NodeHandle nh;

  // Subscriber:
  ros_wrapper::ROSSubscriber ros_sub(&nh);
  auto q1 = ros_sub.add_imu_subscriber("/gx5_0/imu/data");
  auto q2 = ros_sub.add_imu_subscriber("/gx5_1/imu/data");
  auto qv = ros_sub.add_differential_drive_velocity_subscriber("/joint_states");
  ros_sub.start_subscribing_thread();
  // TODO: Create robot state system -- initialize all system threads

  NoiseParams params;
  double temp_param = 0.1;
  params.set_gyroscope_noise(temp_param);
  params.set_accelerometer_noise(temp_param);
  params.set_gyroscope_bias_noise(temp_param);
  params.set_accelerometer_bias_noise(temp_param);
  params.set_augment_noise("contact", temp_param);

  Eigen::Matrix3d measured_velocity_covariance;
  measured_velocity_covariance << 0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01;

  inekf::ErrorType error_type = RightInvariant;
  StateEstimator::StateEstimator state_estimator(params, error_type);

  // Publisher:
  state_estimator.add_imu_propagation(q2);
  state_estimator.add_velocity_correction(qv, measured_velocity_covariance);
  RobotStateQueuePtr robot_state_queue_ptr
      = state_estimator.get_robot_state_queue_ptr();

  ros_wrapper::ROSPublisher ros_pub(&nh, robot_state_queue_ptr);
  ros_pub.start_publishing_thread();

  // block until we stop the ros to print out the value
  while (ros::ok()) {
    // Step behavior
    if (state_estimator.enabled()) {
      state_estimator.run_once();
    } else {
      if (state_estimator.biasInitialized()) {
        state_estimator.initStateByImuAndVelocity();
        state_estimator.enableFilter();
      } else {
        state_estimator.initBias();
      }
    }
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
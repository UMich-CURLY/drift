# \[ROS\] State Estimator Tutorial (InEKF version, IMU + Velocity)

This tutorial will guide you through the process of creating a simple
application using the DRIFT library. It assumes you have a basic knowledge of
C++ and Robot Operating System (ROS). If you are not familiar with ROS, please
refer to the [ROS tutorials](http://wiki.ros.org/ROS/Tutorials). In this tutorial, we focus on using the InEKF version of the state estimator with IMU propagation and velocity correction.

### Step 1: Edit Configs
There are at least two configuration files that need to be edited before running the 
state estimator. The first is the `<propagation>.yaml` file in the `drift/config/filter/inekf/propagation`.
This file contains the settings for the propagation method and users can copy the `imu_propagation.yaml` under that directory
to start with a new set of settings. The second is the `<correction>.yaml` file in the `drift/config/filter/inekf/correction`.
This file contains the settings for the correction method. The `velocity_correction.yaml` file is a good example to start with if a user is interested 
in using velocity message to perform correction. There is also a `legged_kinematics_correction.yaml` file that contains the settings for the legged kinematics correction method. All the example config files contains all the informations needed to run the state estimator with corresponding propagation or correction method.

### Step 2: Create a new case with existing propagation and correction methods
Users can create a new case by following the comments in the `drift/ROS/drift/examples` directory. Let's take the `husky.cpp` as an example. The full file looks like:
```cpp
#include <ros/ros.h>
#include <iostream>

#include "communication/ros_publisher.h"
#include "communication/ros_subscriber.h"
#include "drift/estimator/inekf_estimator.h"

using namespace std;
using namespace state;
using namespace estimator;


int main(int argc, char** argv) {
  /// TUTORIAL: Initialize ROS node
  ros::init(argc, argv, "robot_state_est");

  std::cout << "The subscriber is on!" << std::endl;

  /// TUTORIAL: Initialize ROS node handle. ROS handle handles the
  /// start/shutdown for us
  ros::NodeHandle nh;

  /// TUTORIAL: Create a ROS subscriber
  ros_wrapper::ROSSubscriber ros_sub(&nh);

  /// TUTORIAL: Add a subscriber for IMU data and get its queue and mutex
  auto qimu_and_mutex = ros_sub.AddIMUSubscriber("/gx5_1/imu/data");
  auto qimu = qimu_and_mutex.first;
  auto qimu_mutex = qimu_and_mutex.second;

  /// TUTORIAL: Add a subscriber for velocity data and get its queue and mutex
  auto qv_and_mutex
      = ros_sub.AddDifferentialDriveVelocitySubscriber("/joint_states");
  auto qv = qv_and_mutex.first;
  auto qv_mutex = qv_and_mutex.second;

  /// TUTORIAL: Start the subscriber thread
  ros_sub.StartSubscribingThread();

  /// TUTORIAL: Define some configurations for the state estimator
  inekf::ErrorType error_type = LeftInvariant;
  YAML::Node config_ = YAML::LoadFile(
      "config/filter/inekf/propagation/husky_imu_propagation.yaml");
  bool enable_imu_bias_update
      = config_["settings"]["enable_imu_bias_update"].as<bool>();

  /// TUTORIAL: Create a state estimator
  InekfEstimator inekf_estimator(error_type, enable_imu_bias_update);

  /// TUTORIAL: Add a propagation and correction(s) methods to the state
  /// estimator. Here is an example of IMU propagation and velocity correction
  /// for Husky robot
  inekf_estimator.add_imu_propagation(
      qimu, qimu_mutex,
      "config/filter/inekf/propagation/husky_imu_propagation.yaml");
  inekf_estimator.add_velocity_correction(qv, qv_mutex,
                                          "config/filter/inekf/correction/"
                                          "husky_velocity_correction.yaml");

  /// TUTORIAL: Get the robot state queue and mutex from the state estimator
  RobotStateQueuePtr robot_state_queue_ptr
      = inekf_estimator.get_robot_state_queue_ptr();
  std::shared_ptr<std::mutex> robot_state_queue_mutex_ptr
      = inekf_estimator.get_robot_state_queue_mutex_ptr();

  /// TUTORIAL: Create a ROS publisher and start the publishing thread
  ros_wrapper::ROSPublisher ros_pub(&nh, robot_state_queue_ptr,
                                    robot_state_queue_mutex_ptr);
  ros_pub.StartPublishingThread();

  /// TUTORIAL: Run the state estimator. Initialize the bias first, then
  /// initialize the state. After that, the state estimator will be enabled.
  /// The state estimator should be run in a loop. Users can use RVIZ to
  /// visualize the path. The path topic will be something like
  /// "/robot/*/path"
  while (ros::ok()) {
    // Step behavior
    if (inekf_estimator.is_enabled()) {
      inekf_estimator.RunOnce();
    } else {
      if (inekf_estimator.BiasInitialized()) {
        inekf_estimator.InitState();
      } else {
        inekf_estimator.InitBias();
      }
    }
    ros::spinOnce();
  }

  return 0;
}
```

Let's go through the code step by step.

#### 2.1 Include necessary libraries
The first thing we need to do is to include necessary libraries:
```cpp
#include <ros/ros.h> // ROS
#include <iostream>  // std::cout

#include "communication/ros_publisher.h"  // ROS wrapper for publisher
#include "communication/ros_subscriber.h" // ROS wrapper for subscriber
#include "drift/estimator/inekf_estimator.h"   // InEKF estimator, all the propagation and correction methods are also included in this header file
```
#### 2.2 Create subscribers to receive sensor data
The next step is to create subscribers to receive sensor data. To use ROS, we first need to initialize ROS node and a ROS node handler.
```cpp
ros::init(argc, argv, "robot_state_est");
ros::NodeHandle nh;

```

Then we can initilize the ROS subscriber wrapper and add topics to it. For example, if we want to use IMU data to perform propagation, we can create a subscriber to receive IMU data:
```cpp
// Create a ROS subscriber class
ros_wrapper::ROSSubscriber ros_sub(&nh);
// Create a subscriber to receive IMU data
IMUQueuePair qimu_and_mutex = ros_sub.AddIMUSubscriber("/gx5_1/imu/data");
// Get the pointer to IMU queue and mutex for later use
IMUQueuePtr qimu = qimu_and_mutex.first;
std::shared_ptr<std::mutex> qimu_mutex = qimu_and_mutex.second;
```

Similarly, to add a velocity subscriber, we can do:
```cpp
VelocityQueuePair qv_and_mutex = ros_sub.AddDifferentialDriveVelocitySubscriber("/joint_states");
VelocityQueuePtr qv = qv_and_mutex.first;
std::shared_ptr<std::mutex> qv_mutex = qv_and_mutex.second;
```

The `AddDifferentialDriveVelocitySubscriber(<topic>)` function above will call the `DifferentialDriveVelocityCallback` function in the `ros_subscriber.cpp` file. This function will convert `joint_states` messages into body velocity according to robot's kinematic model. In our case, **Clearpath Husky robot** is a four-wheel-drived robot and the formula to convert `joint_states` messages into body velocity is:
```cpp
// Husky robot's kinematic model
double vr = (right_front_wheel_vel + right_rear_wheel_vel) / 2.0 * wheel_radius;
double vl = (left_front_wheel_vel + left_rear_wheel_vel) / 2.0 * wheel_radius;
double vx = (vr + vl) / 2.0; // Body velocity in x direction
```
If users want to use other robots, they can follow the `DifferentialDriveVelocityCallback` function and create a new function to fit their robots' kinematic model.

After adding all the subscribers, we can start the subscriber thread to avoid traffic jam:
```cpp
ros_sub.StartSubscribingThread();
```

#### 2.3 Create a state estimator
The next step is to create a state estimator. In this example, we will use the InEKF estimator. To create an InEKF estimator, we need to create a propagation method and a correction method. The propagation method is created by calling the `add_<propagation_method>` function in the `inekf_estimator.cpp` file. The correction method is created by calling the `add_<correction_method>` function in the `inekf_estimator.cpp` file. For example, if we want to use IMU data to perform propagation and use velocity data to perform correction, we can do:
```cpp
// Define some config settings for the state estimator
inekf::ErrorType error_type = LeftInvariant;
YAML::Node config_ = YAML::LoadFile(
    "config/filter/inekf/propagation/husky_imu_propagation.yaml");
bool enable_imu_bias_update
    = config_["settings"]["enable_imu_bias_update"].as<bool>();

// Create an InEKF estimator
InekfEstimator inekf_estimator(error_type, enable_imu_bias_update);

// Add a propagation and correction(s) methods
inekf_estimator.add_imu_propagation(qimu, qimu_mutex, "config/filter/inekf/propagation/husky_imu_propagation.yaml");
inekf_estimator.add_velocity_correction(qv, qv_mutex, "config/filter/inekf/correction/husky_velocity_correction.yaml");

// Get the robot state queue and mutex from the state estimator for later use
RobotStateQueuePtr robot_state_queue_ptr = inekf_estimator.get_robot_state_queue_ptr();
std::shared_ptr<std::mutex> robot_state_queue_mutex_ptr = inekf_estimator.get_robot_state_queue_mutex_ptr();
```

#### 2.4 Create publisher to publish state estimation results
We have a ROS publisher wrapper to publish state estimation results. To use it, we need to create a publisher and start the publishing thread:
```cpp
// Create a ROS publisher and start the publishing thread
ros_wrapper::ROSPublisher ros_pub(&nh, robot_state_queue_ptr, robot_state_queue_mutex_ptr);
ros_pub.StartPublishingThread();
```
This publisher will publish the robot state estimation results to the topic `/robot/<robot_name>/state`. Users can use RVIZ to visualize the path. The path topic will be something like `/robot/*/path`.

### Step 3: Run the state estimator (InEKF version)
Now almost everything is settled. The last step will be running the state estimator. The state estimator should be run in a loop like the following:
```cpp
// Run the state estimator
while (ros::ok()) {
  // Step behavior
  if (inekf_estimator.is_enabled()) {
    inekf_estimator.RunOnce();
  } else {
    if (inekf_estimator.BiasInitialized()) {
      inekf_estimator.InitState();
    } else {
      inekf_estimator.InitBias();
    }
  }
  ros::spinOnce();
}

return 0; // Exit
```
In the loop above, we first check if the state estimator is **enabled**. By saying **enabled**, we mean if the necessary biases and initial state are initialized so that the estimator is ready to run. If the estimator is enabled, we call the `RunOnce()` function to perform one step of propagation and correction. If it is not enabled, we would initialize the biases and initial state according to user's settings.

After writing up your own case and adding it to `drift/ROS/drift/CMakeLists.txt`, you can run the state estimator by:
```bash
rosrun drift <YOUR_CASE>
```

Enjoy!







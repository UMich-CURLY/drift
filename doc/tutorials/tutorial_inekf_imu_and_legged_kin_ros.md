# \[ROS\] State Estimator Tutorial (InEKF version, IMU + Legged Kinematics)

This tutorial will guide you through the process of creating a simple
application using the DRIFT library. It assumes you have a basic knowledge of
C++ and Robot Operating System (ROS). If you are not familiar with ROS, please
refer to the [ROS tutorials](http://wiki.ros.org/ROS/Tutorials). In this tutorial we focus on using the InEKF version of the state estimator with IMU propagation and legged kinematics correction.

### Step 1: Edit Configs
There are at least two configuration files that need to be edited before running the 
state estimator. The first is the `<propagation>.yaml` file in the `drift/config/filter/inekf/propagation`.
This file contains the settings for the propagation method and users can copy the `imu_propagation.yaml` under that directory
to start with a new set of settings. The second is the `<correction>.yaml` file in the `drift/config/filter/inekf/correction`. The `legged_kinematics_correction.yaml` file contains the settings for the legged kinematics correction method. All the example config files contains all the informations needed to run the state estimator with corresponding propagation or correction method.

### Step 2: Create a new case with existing propagation and correction methods
Users can create a new case by following the comments in the `drift/ROS/drift/examples` directory. Let's take the `mini_cheetah.cpp` as an example. The full file looks like:
```cpp
/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   mini_cheetah.cpp
 *  @author Tingjun Li
 *  @brief  Test file for Mini-Cheetah robot (IMU propagation + Legged
 *Kinematics Correction)
 *  @date   March 20, 2023
 **/

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
  std::cout << "Subscribing to imu channel..." << std::endl;
  auto qimu_and_mutex = ros_sub.AddIMUSubscriber("/Imu");
  auto qimu = qimu_and_mutex.first;
  auto qimu_mutex = qimu_and_mutex.second;

  /// TUTORIAL: Add a subscriber for legged kinematics data and get its queue
  std::cout << "Subscribing to joint_states and contact channel..."
            << std::endl;
  auto qkin_and_mutex
      = ros_sub.AddMiniCheetahKinematicsSubscriber("/Contacts", "/JointState");
  auto qkin = qkin_and_mutex.first;
  auto qkin_mutex = qkin_and_mutex.second;

  /// TUTORIAL: Start the subscriber thread
  ros_sub.StartSubscribingThread();

  /// TUTORIAL: Define some configurations for the state estimator
  inekf::ErrorType error_type = LeftInvariant;
  YAML::Node config_ = YAML::LoadFile(
      "config/filter/inekf/propagation/mini_cheetah_imu_propagation.yaml");
  bool enable_imu_bias_update
      = config_["settings"]["enable_imu_bias_update"].as<bool>();

  /// TUTORIAL: Create a state estimator
  InekfEstimator inekf_estimator(error_type, enable_imu_bias_update);

  /// TUTORIAL: Add a propagation and correction(s) to the state estimator
  // Mini Cheetah's setting:
  inekf_estimator.add_imu_propagation(
      qimu, qimu_mutex,
      "config/filter/inekf/propagation/mini_cheetah_imu_propagation.yaml");
  inekf_estimator.add_legged_kinematics_correction(
      qkin, qkin_mutex,
      "config/filter/inekf/correction/"
      "mini_cheetah_legged_kinematics_correction.yaml");

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

#### 2.1 Include necessary librarys
The first thing we need to do is to include necessary librarys:
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
IMUQueuePair qimu_and_mutex = ros_sub.AddIMUSubscriber("/Imu");
// Get the pointer to IMU queue and mutex for later use
IMUQueuePtr qimu = qimu_and_mutex.first;
std::shared_ptr<std::mutex> qimu_mutex = qimu_and_mutex.second;
```

Similarly, to add a velocity subscriber, we can do:
```cpp
LeggedKinQueuePair qkin_and_mutex
      = ros_sub.AddMiniCheetahKinematicsSubscriber("/Contacts", "/JointState");
LeggedKinQueuePtr qkin = qkin_and_mutex.first;
std::shared_ptr<std::mutex> qkin_mutex = qkin_and_mutex.second;
```

The `AddMiniCheetahKinematicsSubscriber(<contact_topic>, <joint_state_topic>)` will perform the following steps:
1. Create an ApproximateTime filter to synchronize the contact and joint state data.
2. Call the `ROSSubscriber::MiniCHeetahKinCallBack` to process the synchronized data and generates legged kinematics message required by the state estimator.

**REMINDER:** Users needs to create a new subscriber as well as its callback function for a new robot. They also need to create their own kinematics measurement file resembles to `drift/include/kinematics/mini_cheetah_kinematics.h` and `drift/src/kinematics/mini_cheetah_kinematics.cpp`.

After adding all the subscribers, we can start the subscriber thread to avoid traffic jam:
```cpp
ros_sub.StartSubscribingThread();
```

#### 2.3 Create a state estimator
The next step is to create a state estimator. In this example, we will use the InEKF estimator. To create an InEKF estimator, we need to create a propagation method and a correction method. The propagation method is created by calling the `add_<propagation_method>` function in the `inekf_estimator.cpp` file. The correction method is created by calling the `add_<correction_method>` function in the `inekf_estimator.cpp` file. For example, if we want to use IMU data to perform propagation and use velocity data to perform correction, we can do:
```cpp
// Define some configurations for the state estimator
inekf::ErrorType error_type = LeftInvariant;
YAML::Node config_ = YAML::LoadFile("config/filter/inekf/propagation/mini_cheetah_imu_propagation.yaml");
bool enable_imu_bias_update = config_["settings"]["enable_imu_bias_update"].as<bool>();
    
// Create an InEKF estimator
InekfEstimator inekf_estimator(error_type, enable_imu_bias_update);

// Add a propagation and correction(s) methods
inekf_estimator.add_imu_propagation(
      qimu, qimu_mutex,
      "config/filter/inekf/propagation/mini_cheetah_imu_propagation.yaml");
inekf_estimator.add_legged_kinematics_correction(
    qkin, qkin_mutex,
    "config/filter/inekf/correction/mini_cheetah_legged_kinematics_correction.yaml");

// Get the robot state queue and mutex from the state estimator for later use
RobotStateQueuePtr robot_state_queue_ptr = inekf_estimator.get_robot_state_queue_ptr();
std::shared_ptr<std::mutex> robot_state_queue_mutex_ptr = inekf_estimator.get_robot_state_queue_mutex_ptr();
```
**REMINDER:** If users have different legged robots, they need to create their own kinematics measurement file resembles to `drift/include/kinematics/mini_cheetah_kinematics.h` and `drift/src/kinematics/mini_cheetah_kinematics.cpp`. These two files as well as their corresponding robot kinematics files are used to generate kinematics information needed by the filter. Once these files are settled, the our legged kinematics correction method can be used for user defined legged robots.

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







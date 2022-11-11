# CURLY State Estimator
**Authors:** 

# 1. License
CURLY State Estimator is released under a [GPLv3 license](https://github.com/UMich-CURLY/curly_state_estimator/blob/main/LICENSE). 

# 2. Prerequisites
We have tested the library in **Ubuntu 20.04** and **22.04**, but it should be easy to compile in other platforms.

## C++17 Compiler

## Eigen3
Required by header files. Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

## ROS1 or ROS2 (optional)
Building with ROS is optional. For ROS1, version Noetic was used in testing. For ROS2, verison Foxy was used in testing.

# 3. Building CURLY State Estimator library

Clone the repository:
```
git clone https://github.com/UMich-CURLY/curly_state_estimator.git
```
[WIP]


# 4. ROS1

1. Add the path including */ROS/curly_state_estimator* to the ROS_PACKAGE_PATH environment variable. Open .bashrc file and add at the end the following line. Replace PATH by the folder where you cloned curly_state_estimator:

  ```
  export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/curly_state_estimator/ROS/curly_state_estimator
  ```
  
2. Execute `build_ros.sh` script:

  ```
  chmod +x build_ros.sh
  ./build_ros.sh
  ```
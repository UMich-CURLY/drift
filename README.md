# CURLY State Estimator
**Authors:** 

# 1. License
CURLY State Estimator is released under a [GPLv3 license](https://github.com/UMich-CURLY/curly_state_estimator/blob/main/LICENSE). 


# 2. Prerequisites
We have tested the library in **Ubuntu 20.04** and **22.04**, but it should be easy to compile in other platforms.

## C++11 Compiler
We use the threading functionalities of C++11.

## Eigen3
Required by header files. Download and install instructions can be found at: http://eigen.tuxfamily.org. **Requires at least 3.1.0**.

## ROS1 or ROS2 (optional)
Building with ROS1 or ROS2 is optional. Build instructions are found below.

# 3. Building CURLY State Estimator library

Clone the repository:
```
git clone https://github.com/UMich-CURLY/curly_state_estimator.git
```
Create another directory which we will name 'build' and use cmake and make to compile an build project:

```
mkdir build
cd build
cmake ..
make -j4
```

# 4. ROS
### Building the ROS1 robot_state_est nodes
1. Add `/ROS/curly_state_estimator` to the `ROS_PACKAGE_PATH` environment variable. Open your ~/.bashrc file in a text editor and add the following line to the end. Replace PATH/TO with the directory path to where you cloned curly_state_estimator:

  ```
  export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/TO/curly_state_estimator/ROS/curly_state_estimator
  ```
  
2. Execute `build_ros.sh` script:

  ```
  chmod +x build_ros.sh
  ./build_ros.sh
  ```
# DRIFT
**Authors: Tzu-Yuan Lin, Tingjun Li, Jonathan Tong, and Justin Yu** 

# 1. License
DRIFT is released under a [GPLv3 license](https://github.com/UMich-CURLY/drift/blob/main/LICENSE). 


# 2. Dependencies
We have tested the library in **Ubuntu 20.04** and **22.04**, but it should be easy to compile in other platforms.

> ### C++17 Compiler
We use the threading functionalities of C++17.


> ### Eigen3
Required by header files. Download and install instructions can be found at: http://eigen.tuxfamily.org. **Requires at least 3.1.0**.

> ### Yaml-cpp
Required by header files. Download and install instructions can be found at: https://github.com/jbeder/yaml-cpp.

> ### ROS1 or ROS2 (optional)
Building with ROS1 or ROS2 is optional. Instructions are [found below](https://github.com/UMich-CURLY/drift/tree/main#4-ros).

# 3. Building DRIFT library

Clone the repository:
```
git clone https://github.com/UMich-CURLY/drift.git
```
Create another directory which we will name 'build' and use cmake and make to compile an build project:

```
mkdir build
cd build
cmake ..
make -j4
```

# 4. ROS
## Building the ROS1 robot_state_est nodes
1. Add `/ROS/drift` to the `ROS_PACKAGE_PATH` environment variable. Open your ~/.bashrc file in a text editor and add the following line to the end. Replace PATH/TO with the directory path to where you cloned drift:

  ```
  export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/TO/drift/ROS/drift
  ```

  Then
  ```
  source ~/.bashrc
  ```
  
2. Execute `build_ros.sh` script in the repository root directory:

  ```
  cd <PATH>/<TO>/drift
  chmod +x build_ros.sh
  ./build_ros.sh
  ```

## Run examples:
**Clearpath Husky robot:**
```
rosrun drift ros_comm_test
```

**Fetch robot:**
```
rosrun drift fetch_test
```

**MIT mini-cheetah robot:**
```
rosrun drift leg_kin_test
```

## Run the repo with your own settings:
Users can add configs inside `config/filter/inekf/` directory. Configs in `propagation/` stores settings related to propagation methods, e.g., `mini_cheetah_imu_propagation.yaml` includes all the settings we need to perform imu propagation in the filter. Configs
in `correction` stores settings related to correction methods. For example, velocity correction method settings can refer to `fetch_velocity_correction.yaml` or `velocity_correction.yaml`, while legged kinematics correction method settings can refer to 
`mini_cheetah_legged_kinematics_correction.yaml`. 

After editting the config files, one can easily follow tutorial comments in examples in `ROS/examples` to create new cases!

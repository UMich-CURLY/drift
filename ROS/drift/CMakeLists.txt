cmake_minimum_required(VERSION 3.14)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# documentation for rosbuild macros: http://wiki.ros.org/rosbuild/CMakeLists
rosbuild_init()

# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/bin)
IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  trajectory_msgs
  message_generation
  message_filters
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED timer chrono)
find_package(yaml-cpp REQUIRED)

message("Eigen3 include dir: ${EIGEN3_INCLUDE_DIR}")
message("Boost include dir: ${Boost_INCLUDE_DIRS}")
message("Drift include dir: ${drift_INCLUDE_DIRS}")
message("Yaml-cpp include dir: ${YAML_CPP_INCLUDE_DIR}")
message("catkin include dir: ${catkin_INCLUDE_DIRS}")

include_directories(
  SYSTEM # suppress warnings from 3rd party libs that we can't control
  ${PROJECT_SOURCE_DIR}/../../include
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${drift_INCLUDE_DIRS}
)

message("baseline include dir: ${PROJECT_SOURCE_DIR}/../../../baselines/include")

set(LIBS
  ${EIGEN3_LIBS}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Boost_GENERAL}
)

# Kinematic files for mini cheetah robot:
file(GLOB MINI_CHEETAH_KIN_SRC_FILES ${PROJECT_SOURCE_DIR}/../../src/kinematics/robots/mini_cheetah/*.cpp)

rosbuild_add_library(drift
  ${PROJECT_SOURCE_DIR}/../../src/state/robot_state.cpp
  ${PROJECT_SOURCE_DIR}/../../src/estimator/inekf_estimator.cpp
  ${PROJECT_SOURCE_DIR}/../../src/math/lie_group.cpp
  ${PROJECT_SOURCE_DIR}/../../src/math/se_k_3.cpp
  ${PROJECT_SOURCE_DIR}/../../src/filter/base_propagation.cpp
  ${PROJECT_SOURCE_DIR}/../../src/filter/base_correction.cpp
  ${PROJECT_SOURCE_DIR}/../../src/filter/inekf/correction/legged_kinematics_correction.cpp
  ${PROJECT_SOURCE_DIR}/../../src/filter/inekf/correction/velocity_correction.cpp
  ${PROJECT_SOURCE_DIR}/../../src/filter/inekf/correction/position_correction.cpp
  ${PROJECT_SOURCE_DIR}/../../src/filter/inekf/propagation/imu_propagation.cpp
  ${PROJECT_SOURCE_DIR}/../../src/filter/inekf/inekf.cpp
  ${PROJECT_SOURCE_DIR}/../../src/imu_filter/imu_ang_vel_ekf.cpp
  ${PROJECT_SOURCE_DIR}/../../src/measurement/measurement.cpp
  ${PROJECT_SOURCE_DIR}/../../src/measurement/legged_kinematics.cpp
  ${PROJECT_SOURCE_DIR}/../../src/kinematics/mini_cheetah_kinematics.cpp
  ${MINI_CHEETAH_KIN_SRC_FILES}
)

set(ros_subscriber ${PROJECT_SOURCE_DIR}/src/communication/ros_subscriber.cpp)
set(ros_publisher ${PROJECT_SOURCE_DIR}/src/communication/ros_publisher.cpp)
rosbuild_add_library(ros_communication_lib
  ${ros_subscriber}
  ${ros_publisher}
)

# # baselines
include_directories(
  ${PROJECT_SOURCE_DIR}/../../baselines/include)
rosbuild_add_library(baselines
  ${PROJECT_SOURCE_DIR}/../../baselines/src/wheel_odometry/wheel_odometry.cpp)

# example for legged robot kinemtaics correction:
rosbuild_add_executable(mini_cheetah
  ${PROJECT_SOURCE_DIR}/examples/mini_cheetah.cpp)

target_link_libraries(mini_cheetah
  ${LIBS}
  ros_communication_lib
  drift
  yaml-cpp
)

# # temporary test for ros communicator: subscriber + publisher
rosbuild_add_executable(husky
  ${PROJECT_SOURCE_DIR}/examples/husky.cpp)

target_link_libraries(husky
  ${LIBS}
  ros_communication_lib
  drift
  yaml-cpp
)

# # temporary test for fetch
rosbuild_add_executable(fetch
  ${PROJECT_SOURCE_DIR}/examples/fetch.cpp)

target_link_libraries(fetch
  ${LIBS}
  ros_communication_lib
  drift
  yaml-cpp
)

# # temporary test for fetch
rosbuild_add_executable(fetch_gyro_filter
  ${PROJECT_SOURCE_DIR}/examples/fetch_gyro_filter.cpp)

target_link_libraries(fetch_gyro_filter
  ${LIBS}
  ros_communication_lib
  drift
  yaml-cpp
)

rosbuild_add_executable(husky_gyro_filter
  ${PROJECT_SOURCE_DIR}/examples/husky_gyro_filter.cpp)

target_link_libraries(husky_gyro_filter
  ${LIBS}
  ros_communication_lib
  drift
  yaml-cpp
)

rosbuild_add_executable(husky_odom
  ${PROJECT_SOURCE_DIR}/examples/husky_odom.cpp)

target_link_libraries(husky_odom
  ${LIBS}
  ros_communication_lib
  drift
  yaml-cpp
)

rosbuild_add_executable(wamv_gtodom_vel
  ${PROJECT_SOURCE_DIR}/examples/wamv_gtodom_vel.cpp)

target_link_libraries(wamv_gtodom_vel
  ${LIBS}
  ros_communication_lib
  drift
  yaml-cpp
)

rosbuild_add_executable(wamv_gtodom
  ${PROJECT_SOURCE_DIR}/examples/wamv_gtodom.cpp)

target_link_libraries(wamv_gtodom
  ${LIBS}
  ros_communication_lib
  drift
  yaml-cpp
)

rosbuild_add_executable(wamv_gps
  ${PROJECT_SOURCE_DIR}/examples/wamv_gps.cpp)

target_link_libraries(wamv_gps
  ${LIBS}
  ros_communication_lib
  drift
  yaml-cpp
)

rosbuild_add_executable(girona500
  ${PROJECT_SOURCE_DIR}/examples/girona500.cpp)

target_link_libraries(girona500
  ${LIBS}
  ros_communication_lib
  drift
  yaml-cpp
)

rosbuild_add_executable(husky_double_imu_filter
  ${PROJECT_SOURCE_DIR}/examples/husky_double_imu_filter.cpp)

target_link_libraries(husky_double_imu_filter
  ${LIBS}
  ros_communication_lib
  drift
  yaml-cpp
)

rosbuild_add_executable(neya
  ${PROJECT_SOURCE_DIR}/examples/neya.cpp)

target_link_libraries(neya
  ${LIBS}
  ros_communication_lib
  drift
  yaml-cpp
)

rosbuild_add_executable(neya_gyro_filter
  ${PROJECT_SOURCE_DIR}/examples/neya_gyro_filter.cpp)

target_link_libraries(neya_gyro_filter
  ${LIBS}
  ros_communication_lib
  drift
  yaml-cpp
)

# ## baselines ###
rosbuild_add_executable(fetch_wheel_odom
  ${PROJECT_SOURCE_DIR}/baselines/fetch_wheel_odom.cpp)

target_link_libraries(fetch_wheel_odom
  ${LIBS}
  ros_communication_lib
  drift
  yaml-cpp
  baselines
)

rosbuild_add_executable(husky_wheel_odom
  ${PROJECT_SOURCE_DIR}/baselines/husky_wheel_odom.cpp)

target_link_libraries(husky_wheel_odom
  ${LIBS}
  ros_communication_lib
  drift
  yaml-cpp
  baselines
)
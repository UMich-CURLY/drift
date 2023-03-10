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

# catkin_package(
#   INCLUDE_DIRS include
#   CATKIN_DEPENDS roscpp std_msgs geometry_msgs trajectory_msgs message_generation message_filters
# )

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(yaml-cpp REQUIRED)

message("Eigen3 include dir: ${EIGEN3_INCLUDE_DIR}")
message("Boost include dir: ${Boost_INCLUDE_DIRS}")
message("Yaml-cpp include dir: ${YAML_CPP_INCLUDE_DIR}")
message("catkin include dir: ${catkin_INCLUDE_DIRS}")

include_directories(
${PROJECT_SOURCE_DIR}/../../include
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${YAML_CPP_INCLUDE_DIR}
${Boost_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS}
)

set(LIBS 
  ${EIGEN3_LIBS}
  ${catkin_LIBRARIES}
)

file(GLOB_RECURSE SRC_FILES ${PROJECT_SOURCE_DIR}/src/*.cpp)
file(GLOB MINI_CHEETAH_KIN_SRC_FILES ${PROJECT_SOURCE_DIR}/../../src/kinematics/robots/mini_cheetah/*.cpp)

rosbuild_add_library(mini_cheetah_kin_lib
  ${MINI_CHEETAH_KIN_SRC_FILES}
  ${PROJECT_SOURCE_DIR}/../../src/kinematics/mini_cheetah_kinematics.cpp
)

rosbuild_add_library(filter_lib
  # State Estimator
  ${PROJECT_SOURCE_DIR}/../../src/state_estimator.cpp
  # Filter
  ${PROJECT_SOURCE_DIR}/../../src/filter/base_propagation.cpp
  ${PROJECT_SOURCE_DIR}/../../src/filter/base_correction.cpp
  ${PROJECT_SOURCE_DIR}/../../src/filter/inekf/correction/legged_kinematics_correction.cpp
  ${PROJECT_SOURCE_DIR}/../../src/filter/inekf/correction/velocity_correction.cpp
  ${PROJECT_SOURCE_DIR}/../../src/filter/inekf/propagation/imu_propagation.cpp
  ${PROJECT_SOURCE_DIR}/../../src/filter/inekf/inekf.cpp
  # State
  ${PROJECT_SOURCE_DIR}/../../src/state/robot_state.cpp
  ${PROJECT_SOURCE_DIR}/../../src/state/se_k_3.cpp
  # Math
  ${PROJECT_SOURCE_DIR}/../../src/math/lie_group.cpp
  
)

rosbuild_add_library(measurement_lib
  ${PROJECT_SOURCE_DIR}/../../src/measurement/measurement.cpp
  ${PROJECT_SOURCE_DIR}/../../src/measurement/legged_kinematics.cpp
)

set(ros_subscriber ${PROJECT_SOURCE_DIR}/src/communication/ros_subscriber.cpp)
set(ros_publisher ${PROJECT_SOURCE_DIR}/src/communication/ros_publisher.cpp)

rosbuild_add_library(ros_communication_lib
  ${ros_subscriber}
  ${ros_publisher}
)


# temporary test for subscriber
# rosbuild_add_executable(subscriber_test
# ${PROJECT_SOURCE_DIR}/examples/subscriber_test.cpp)

# rosbuild_add_library(subscriber_test_lib
# ${PROJECT_SOURCE_DIR}/src/communication/ros_subscriber.cpp
# ${PROJECT_SOURCE_DIR}/../../src/measurement/measurement.cpp
# ${PROJECT_SOURCE_DIR}/../../src/measurement/legged_kinematics.cpp
# )

# target_link_libraries(subscriber_test
# ${LIBS}
# subscriber_test_lib
# mini_cheetah_kin_lib
# )

# example for legged robot kinemtaics correction:
rosbuild_add_executable(leg_kin_comm_test
${PROJECT_SOURCE_DIR}/examples/leg_kin_comm_test.cpp)

# rosbuild_add_library(leg_kin_comm_test_lib
# ${PROJECT_SOURCE_DIR}/src/communication/ros_subscriber.cpp
# ${PROJECT_SOURCE_DIR}/../../src/measurement/measurement.cpp
# ${PROJECT_SOURCE_DIR}/../../src/measurement/legged_kinematics.cpp
# )

target_link_libraries(leg_kin_comm_test
${LIBS}
ros_communication_lib
measurement_lib
filter_lib
mini_cheetah_kin_lib
yaml-cpp
)

# # temporary test for ros communicator: subscriber + publisher
rosbuild_add_executable(ros_comm_test
${PROJECT_SOURCE_DIR}/examples/ros_comm_test.cpp)


target_link_libraries(ros_comm_test
${LIBS}
ros_communication_lib
measurement_lib
filter_lib
mini_cheetah_kin_lib
yaml-cpp
)

# # temporary test for fetch
# rosbuild_add_executable(fetch_test
# ${PROJECT_SOURCE_DIR}/examples/fetch_test.cpp)


# target_link_libraries(fetch_test
# ${LIBS}
# ros_communication_lib
# measurement_lib
# filter_lib
# mini_cheetah_kin_lib
# yaml-cpp
# )

# # test for gps vel
rosbuild_add_executable(gps_vel_test
${PROJECT_SOURCE_DIR}/examples/gps_vel_test.cpp)


target_link_libraries(gps_test
${LIBS}
ros_communication_lib
measurement_lib
filter_lib
mini_cheetah_kin_lib
yaml-cpp
)
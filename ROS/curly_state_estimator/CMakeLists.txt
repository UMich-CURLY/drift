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

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)


include_directories(
${PROJECT_SOURCE_DIR}/../../include
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${Boost_INCLUDE_DIRS}
)

set(LIBS 
${EIGEN3_LIBS}
)
file(GLOB_RECURSE SRC_FILES ${PROJECT_SOURCE_DIR}/src/*.cpp)

# Node for robot state estimation
rosbuild_add_executable(robot_state_est
${PROJECT_SOURCE_DIR}/examples/robot_state_est.cpp 
)

target_link_libraries(robot_state_est
${LIBS}
)

# temporary test for subscriber
rosbuild_add_executable(subscriber_test
${PROJECT_SOURCE_DIR}/examples/subscriber_test.cpp)

add_library(subscriber_test_lib
${PROJECT_SOURCE_DIR}/src/communication/ros_subscriber.cpp
${PROJECT_SOURCE_DIR}/../../src/measurement/measurement.cpp)

target_link_libraries(subscriber_test
${LIBS}
subscriber_test_lib
)

# temporary test for ros communicator: subscriber + publisher
rosbuild_add_executable(ros_comm_test
${PROJECT_SOURCE_DIR}/examples/ros_comm_test.cpp)

add_library(ros_comm_test_lib
${PROJECT_SOURCE_DIR}/src/communication/ros_comm_test.cpp
${PROJECT_SOURCE_DIR}/../../src/measurement/measurement.cpp
${PROJECT_SOURCE_DIR}/../../src/state_estimator.cpp
${PROJECT_SOURCE_DIR}/../../src/state/robot_state.cpp)

target_link_libraries(ros_comm_test
${LIBS}
ros_comm_test_lib
)
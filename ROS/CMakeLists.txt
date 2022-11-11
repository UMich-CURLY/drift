cmake_minimum_required(VERSION 3.14)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})


find_package(Eigen3 3.1.0 REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
)

set(LIBS 
${EIGEN3_LIBS}
)

# Node for monocular camera
rosbuild_add_executable(robot_state
src/robot_state.cpp
)

target_link_libraries(robot_state
${LIBS}
)

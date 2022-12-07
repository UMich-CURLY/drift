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

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)


include_directories(
${PROJECT_SOURCE_DIR}/../../include
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
${PROJECT_SOURCE_DIR}/examples/subscriber_test.cpp ${PROJECT_SOURCE_DIR}/src/communication/ros_subscriber.cpp)

target_link_libraries(subscriber_test
${LIBS})
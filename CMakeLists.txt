cmake_minimum_required(VERSION 3.14)

project(curly_state_estimator VERSION 0.1 DESCRIPTION "An InEKF based state estimator")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3")

option(BUILD_EXAMPLES "Build the examples" ON)
option(BUILD_TESTS "Build the test files" ON)
option(BUILD_DOC "Build Doxygen documents" ON)

# #################################################
# Find External Packages             #
# #################################################
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(yaml-cpp REQUIRED)

message("Eigen3 include dir: ${EIGEN3_INCLUDE_DIR}")
message("Boost include dir: ${Boost_INCLUDE_DIRS}")
message("Yaml-cpp include dir: ${YAML_CPP_INCLUDE_DIR}")
# #################################################
# Include Packages                #
# #################################################
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${YAML_CPP_INCLUDE_DIR})
include(FetchContent)

# #################################################
# Project Include Dir              #
# #################################################
include_directories(${PROJECT_SOURCE_DIR}/include)


# #################################################
# Create Library                 #
# #################################################
add_library(robot_state ${PROJECT_SOURCE_DIR}/src/state/robot_state.cpp)
add_library(se_k_3 ${PROJECT_SOURCE_DIR}/src/state/se_k_3.cpp)
add_library(state_estimator ${PROJECT_SOURCE_DIR}/src/state_estimator.cpp)
add_library(lie_group ${PROJECT_SOURCE_DIR}/src/math/lie_group.cpp)
add_library(base_propagation ${PROJECT_SOURCE_DIR}/src/filter/base_propagation.cpp)
add_library(base_correction ${PROJECT_SOURCE_DIR}/src/filter/base_correction.cpp)
add_library(inekf ${PROJECT_SOURCE_DIR}/src/filter/inekf/correction/legged_kinematics_correction.cpp
  ${PROJECT_SOURCE_DIR}/src/filter/inekf/correction/velocity_correction.cpp
  ${PROJECT_SOURCE_DIR}/src/filter/inekf/propagation/imu_propagation.cpp
  ${PROJECT_SOURCE_DIR}/src/filter/inekf/inekf.cpp)
add_library(measurement ${PROJECT_SOURCE_DIR}/src/measurement/measurement.cpp)

# #################################################
# Build Examples                 #
# #################################################
if(BUILD_EXAMPLES)
  message("Building examples...")
  add_subdirectory(examples)
endif(BUILD_EXAMPLES)

# #################################################
# Build Tests                  #
# #################################################
if(BUILD_TESTS)
  message("Building tests...")
  add_subdirectory(tests)
endif(BUILD_TESTS)

# #################################################
# Build Documents                #
# #################################################
if(BUILD_DOC)
  # find doxygen
  find_package(Doxygen QUIET)

  if(DOXYGEN_FOUND)
    message("Building documents...")

    # set input and output files
    set(DOXYGEN_IN ${PROJECT_SOURCE_DIR}/doc/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    # request to configu re the file
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

    # note the option ALL which allows to build the docs together with the application
    add_custom_target(doc_doxygen ALL
      COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
      COMMENT "Generating API documentation with Doxygen"
      VERBATIM)
  else(DOXYGEN_FOUND)
    message(WARNING "Doxygen need to be installed to generate the doxygen documentation")
  endif(DOXYGEN_FOUND)
endif(BUILD_DOC)

# #################################################
# Add Executables                #
# #################################################
